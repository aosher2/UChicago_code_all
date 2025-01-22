from collections import OrderedDict
import time
import numpy as np

import nidaqmx
from nidaqmx.stream_writers import AnalogMultiChannelWriter
from nidaqmx.stream_readers import CounterReader
from nidaqmx.constants import Edge, TriggerType, TaskMode, AcquisitionType, READ_ALL_AVAILABLE
from rpyc.utils.classic import obtain


class nidaqMotionControl():
    DEFAULT_UNITS_DISTANCE = 'um'
    DEFAULT_UNITS_RATE     = 'Hz'
    DEFAULT_UNITS_VOLTAGE  = 'V'

    def __init__(self, x_ch= 'Dev1/ao0', y_ch='Dev1/ao1', ctr_ch='Dev1/ctr0', XperV = 5e-6, YperV = 5e-6):
        '''
        Motion controller for a voltage-driven FSM
        
        Arguments:
                *x_ch: Physical AO DAQ channel corresponding to x-axis of FSM, e.g. 'Dev1/ao0'
                *y_ch: Physical AO DAQ channel corresponding to y-axis of FSM
                *ctr_ch: Channel to count digital pulses on
                *XperV: calibration between voltage applied to the FSM vs movement of confocal spot on sample
                *YperV: calibration between voltage applied to the FSM vs movement of confocal spot on sample
        '''
        self.x_axis = x_ch
        self.y_axis = y_ch
        self.ctr_ch = ctr_ch
        self.XperV  = XperV
        self.YperV  = YperV

        # Just for fun, put all information into the same dictionary
        self.axesDict = {'x-axis' : {'channel' : self.x_axis,
                                     'calibration' : self.XperV
                                    },
                         'y-axis' : {'channel' : self.y_axis,
                                     'calibration' : self.YperV
                                    }}

        self.position = {'x': 0, 'y': 0}
        self.ctrTasks = []
        self.ctr_task = None
        self.ao_task  = None
        self.ai_task = None # For reading current position

    def __enter__(self):
        # Create analog output task
        self.ao_task = nidaqmx.Task('Analog Output to FSM Driver')
        self.ao_task.ao_channels.add_ao_voltage_chan(self.x_axis, name_to_assign_to_channel = 'x AO0')
        self.ao_task.ao_channels.add_ao_voltage_chan(self.y_axis, name_to_assign_to_channel = 'y AO1')
        return self
    
    def __exit__(self, *args):
        # Close all counter tasks
        for ctrTask in self.ctrTasks:
            ctrTask.close()
        # Close the analog output task
        self.ao_task.close()
        return self

    def um_to_V(self, value, axisName):
        if axisName == 'x':
            return value / self.XperV
        elif axisName == 'y':
            return value / self.YperV
        
    def V_to_um(self, value, axisName):
        if axisName == 'x':
            return value * self.XperV
        elif axisName == 'y':
            return value * self.YperV
        
    def move(self, point, points_per_volt = 50):
        '''
        Write analog voltages to the appropriate channels of the NI DAQ to move the FSM to point.

        Note the maximum velocity of movement of the FSM is 2*(10V + (-10V))* 40Hz = 1600V/s. The formula for calculating
        the rate of the ao_task clock is given by: clock_rate = 1600 V/s * 0.75 * points_per_volt * voltage difference between
        initial and final positions. 0.75 is a "fudge factor" to ensure we operate below maximum thermal load.

        Arguments:
                *point: dictionary containig axis names mapped to taret values (in um), e.g. {'x': 0.5, 'y':1.5}
                *points_per_volt: adds additional interpolation points to make movement smooth
        '''
        # Get rid of netrefs
        point = obtain(point)

        # Update current position
        self.position = self.read_current_position()

        # Generate list of voltage steps        
        voltage_array = self.smooth_voltages(self.position, point, points_per_volt)

        # Rate of movement
        dV = self.voltage_distance_between_points(self.position,point)
        if np.isclose(dV,0.0):
            ao_clock_rate = 100 # arbitrary, just not zero
        else:
            ao_clock_rate = 1600 * 0.75 * points_per_volt * dV

        # Configure timing of analog output task
        self.ao_task.timing.cfg_samp_clk_timing(ao_clock_rate,
                                                sample_mode   = AcquisitionType.FINITE,
                                                samps_per_chan = len(voltage_array))
        self.ao_task.triggers.start_trigger.disable_start_trig()
                
        # Create stream writer
        streamWriter = AnalogMultiChannelWriter(self.ao_task.out_stream, auto_start = False)

        # Create buffer for voltages
        buffer = np.ascontiguousarray(voltage_array.transpose(), dtype = float)
        # Stream buffer
        streamWriter.write_many_sample(buffer, timeout=10)

        # Start AO task
        self.ao_task.start()
        
        # Wait until AO task finishes
        self.ao_task.wait_until_done()
        # Stop AO Task
        self.ao_task.stop()

        # Update position
        self.position = point

    def move_relative(self, displacement, points_per_volt=50):
        '''
        Move the FSM to a relative displacement away from its current position.

        Arguments:
                * displacement: dictionary containing axis names mapped to target displacements (in um), e.g. {'x': 0.01, 'y':0.01}
                * points_per_volt: adds additiaonl interpolation points to make movement smooth, fairly arbitrary
        '''
        # Get rid of netrefs
        displacement = obtain(displacement)

        # Update current position
        self.position = self.read_current_position()

        x = self.position['x']
        y = self.position['y']

        dx = displacement['x']
        dy = displacement['y']

        # Check bounds
        if (abs(self.um_to_V(x+dx,'x')) > 10.0 or abs(self.um_to_V(y+dy,'y'))) > 10.0:
            raise ValueError("Relative movement to {}um is out of range. Movement not exectued.".format(str*(x+dx)))
        
        new_point = {'x': x +dx, 'y': y+dy}

        self.move(new_point)

        self.position = new_point


    def oneD_scan(self, initial_point, final_point, num_pixels = 40, scan_rate = 100, avgs_per_pixel = 20, data_channel = 'Dev1/PFI1'):
        '''
        One-dimensional line scan of the FSM. The line scan is composed of (num_pixels) pixels, where the FSM will stay on each pixel for
        a given time (1/scan_rate). The DAQ signal collected on (data_channel) will be sampled (avgs_per_pixel) times for each pixel. This
        makes the DAQ acquisiton rate (implicit in this method) scan_rate * avgs_per_pixel. For each pixel, the DAQ values sampled during the
        FSM's time on this pixel are averaged together and multiplied by the DAQ acquistion rate. This method return a 1D array where each element
        correponds to one pixel and the element value is the count rate (cps, Hz) measured by the DAQ at each pixel.


        Note the maximum velocity of movement of the FSM is 2*(10V + (-10V))* 40Hz = 1600V/s. Set the acquistion rate
        such that each step of the 1D scan does not exceed this velocity.

        Arguments:
                *inital_point: dictionary giving start point for the 1D scan (in um),  e.g. {'x': 0.5, 'y':1.5}
                *final_point: dictionary giving end point for the 1D scan (in um), e.g. {'x': 0.5, 'y':1.5}
                *num_pixels: number of dwell points for the 1D scan
                *scan_rate: rate/dwell time of collection for the 1D scan on each pixel
                *avgs_per_pixel: for each pixel, read the DAQ this many times and average the result
                *data_channel: physical DAQ channel that collects data (default: PFI channel for collecting digital pulses);
                add more data processing to this parameter if other type of data used, e.g. analog voltages
        '''

        # Get rid of netrefs
        initial_point = obtain(initial_point)
        final_point = obtain(final_point)
        num_pixels = obtain(num_pixels)
        scan_rate = obtain(scan_rate)
        avgs_per_pixel = obtain(avgs_per_pixel)
        data_channel = obtain(data_channel)

        # Move FSM to initial point
        #self.move(initial_point, points_per_volt = 100)

        # Generate voltages to represent location of each pixel; each pixel is repeated avgs_per_pixel times; last row is duplicated an extra time
        voltage_array = self.linear_voltages(initial_point, final_point, num_pixels, avgs_per_pixel)

        # Exceed max velocity?
        dV = self.voltage_distance_between_points(initial_point,final_point)
        if dV * scan_rate / num_pixels >= 1600:
            print("Careful! High FSM velocity ({}). Reduce scan_rate.".format(str(dV*scan_rate)))

        # Configure timing of the analog output task
        self.ao_task.timing.cfg_samp_clk_timing(scan_rate * avgs_per_pixel,
                                                sample_mode    = AcquisitionType.FINITE,
                                                samps_per_chan = len(voltage_array))
        

        # Create stream writer for ao voltages
        streamWriter = AnalogMultiChannelWriter(self.ao_task.out_stream, auto_start = False)

        # Create buffer for ao voltages
        ao_buffer = np.ascontiguousarray(voltage_array.transpose(), dtype = np.float64)

        # Stream ao buffer
        streamWriter.write_many_sample(ao_buffer, timeout = 60)

        ############ Connect AO movement to counter ###############
        dev_name = self.x_axis[0:4] # Hard code this in b/d doesn't change much

        # Create a counter task
        self.new_ctr_task(self.ctr_ch, data_channel)

        # Configure timing of the counter task
        self.ctr_task.timing.cfg_samp_clk_timing(scan_rate * avgs_per_pixel,
                                                 source         = '/{}/ao/SampleClock'.format(dev_name),
                                                 sample_mode    = AcquisitionType.FINITE,
                                                 samps_per_chan = len(voltage_array))

        # Set the couter input to trigger (aka start couting) when the AO starts
        self.ctr_task.triggers.arm_start_trigger.dig_edge_src = '/{}/ao/StartTrigger'.format(dev_name)
        self.ctr_task.triggers.arm_start_trigger.trig_type    = nidaqmx.constants.TriggerType.DIGITAL_EDGE

        # Create stream reader for ctr value
        streamReader = CounterReader(self.ctr_task.in_stream)

        # Create buffer for counter
        ctr_buffer = np.ascontiguousarray(np.zeros(len(voltage_array), dtype= np.uint32))
                                          
        # Start the line scan (ctr triggers when ao starts)
        self.ctr_task.start()
        self.ao_task.start()

        # Read the counter
        streamReader.read_many_sample_uint32(ctr_buffer,
                                             number_of_samples_per_channel = nidaqmx.constants.READ_ALL_AVAILABLE,
                                             timeout = 60)
        
        # Wait until AO task finishes
        self.ao_task.wait_until_done()

        # Stop the tasks
        self.ctr_task.stop()
        self.ao_task.stop()

        # Save final position
        self.position = final_point

        ############################ Data processing ##################################
        counts = np.diff(ctr_buffer)  # Raw counts where each element corresponds to a sample by the DAQ

        bin_counts = np.reshape(counts, (num_pixels, avgs_per_pixel)) # Bin counts corresponding to each unique voltage step

        avg_counts = np.mean(bin_counts, axis=1) # Average together counts in each bin

        # Get rate from counts
        rates = avg_counts  * scan_rate * avgs_per_pixel

        return rates
    
    def twoD_scan(self, initial_point, final_point, num_pixels_x, num_pixels_y, scan_rate, avgs_per_pixel, data_channel):
        '''
        Perform a 2D scan of an area specified by top left corner initial_point and bottom right corner final_point. Image will be formed with
        num_pixels_x and num_pixels_y pixels in each respective direction. 

        The 2D scan is rastered; that is, the scan will snake from the top left corner to the top right corner, hop down (-y) one pixel and return
        scanning to the right. This pattern repeats until the end of the raster scan.

        A 2D scan is formed by calling oneD_scan(num_pixels = num_pixels_x) num_pixels_y number of times. Data is collected on each pixel for 
        1/scan_rate seconds. Data is collected on DAQ channel data_channel (currently configured to only sample digital inputs/TTL pulses) avgs_per_pixel
        number of times per pixel. The raw counts are then averaged over each pixel and normalized to time.

        This method returns a 2D numpy array that is num_pixels_x by num_pixels_y in size. Each element is the cps (Hz) recorded by the DAQ
        during the time the FSM dwelled at each pixel.

        NOTE: if you wish to send data to the GUI line-by-line, you cannot use this method. Instead, build a for loop like the one 
        in this method in your own fsmScan code.

        Arguments:
                *inital_point: dictionary giving top left corner for the 2D scan (in um),  e.g. {'x': -10, 'y': 10}
                *final_point: dictionary giving bottom right corner for the 2D scan (in um), e.g. {'x': 10, 'y':-10}
                *num_pixels_x: number of dwell points for the 2D scan in the x-direction
                *num_pixels_y: number of dwell points for the 2D scan in the y-direction
                *scan_rate: rate/dwell time of collection for the 2D scan on each pixel
                *avgs_per_pixel: for each pixel, read the DAQ this many times and average the result
                *data_channel: physical DAQ channel that collects data (default: PFI channel for collecting digital pulses)
        '''
        # Get rid of netrefs
        initial_point = obtain(initial_point)
        final_point = obtain(final_point)
        num_pixels_x = obtain(num_pixels_x)
        num_pixels_y = obtain(num_pixels_y)
        scan_rate = obtain(scan_rate)
        avgs_per_pixel = obtain(avgs_per_pixel)
        data_channel = obtain(data_channel)

        rates    = np.zeros((num_pixels_y,num_pixels_x))
        x_min    = initial_point['x']
        x_max    = final_point['x']
        y_values = np.linspace(initial_point['y'],final_point['y'],num_pixels_y)

        for y_value, i in zip(y_values, range(len(y_values))):

            left_point = {'x': x_min, 'y': y_value}
            right_point = {'x': x_max, 'y': y_value}
            # rows going left to right
            if i % 2 == 0:
                rates_in_row = self.oneD_scan(left_point,right_point,num_pixels_x,scan_rate,avgs_per_pixel,data_channel)

            # rows going right to left
            else:
                rates_in_row = self.oneD_scan(right_point,left_point,num_pixels_x,scan_rate,avgs_per_pixel,data_channel)
                rates_in_row = np.flip(rates_in_row)

            rates[i] = rates_in_row

        # Save final position
        self.position = final_point

        return rates

    def read_current_position(self):
        with nidaqmx.Task() as task:
            task = nidaqmx.Task('Analog Input Read of Output to FSM Driver')
            #task.ai_channels.add_ai_voltage_chan('Dev1/AI1')
            task.ai_channels.add_ai_voltage_chan('Dev1/_ao0_vs_aognd', min_val = -10.0, max_val = 10.0)
            task.ai_channels.add_ai_voltage_chan('Dev1/_ao1_vs_aognd', min_val = -10.0, max_val = 10.0)

            data = task.read()
            task.wait_until_done()
            task.stop()
            task.close()
        return {'x':self.V_to_um(data[0],'x'),'y':self.V_to_um(data[1],'y')}

    def check_bounds(self, point):
        '''
        Make sure that the point is within the voltage limits of the FSM (+/- 10V on either axis).
        
        Arguments:
                *point: dictionary containing axis names mapped to target values (in um), e.g. {'x': 0.5, 'y':1.5}
        '''
        # Get rid of netrefs
        point = obtain(point)

        if (abs(point['x']/self.XperV) or abs(point['y']/self.YperV)) > 10.0:
            raise ValueError("Relative movement to {} is out of range. Movement not exectued.".format(point))
    
    def voltage_distance_between_points(self, initial_point, final_point):
        '''
        Arguments:
                *initial_point: dictionary containig axis names mapped to taret values (in um), e.g. {'x': 0.5, 'y':1.5}
                *final_point:   dictionary containig axis names mapped to taret values (in um), e.g. {'x': 0.5, 'y':1.5}
        '''
        # Get rid of netrefs
        initial_point = obtain(initial_point)
        final_point = obtain(final_point)

        initial_volts = np.array([self.um_to_V(initial_point['x'],'x'), self.um_to_V(initial_point['y'],'y')])
        final_volts   = np.array([self.um_to_V(final_point['x'],'x'),  self.um_to_V(final_point['y']  ,'y')])
        return max(abs(final_volts-initial_volts))


    def smooth_voltages(self, initial_point, final_point, points_per_volt):
        '''
        Generate a numpy array of voltages corresponding to a msooth path from initial_point to final_point.

        Arguments:
                *initial_point: dictionary containig axis names mapped to taret values (in um), e.g. {'x': 0.5, 'y':1.5}
                *final_point:   dictionary containig axis names mapped to taret values (in um), e.g. {'x': 0.5, 'y':1.5}

        Return:
                Numpy array of voltages corresponding to a smooth path from initial_point to final_point
        '''
        # Get rid of netrefs
        initial_point = obtain(initial_point)
        final_point = obtain(final_point)
        points_per_volt = obtain(points_per_volt)

        initial_volts = np.array([self.um_to_V(initial_point['x'],'x'), self.um_to_V(initial_point['y'],'y')])
        final_volts   = np.array([self.um_to_V(final_point['x'],'x'),   self.um_to_V(final_point['y'],'y')  ])

        # Calculate the number of steps to take in this call to move. num_steps is the voltage distance between the initial and final coordinates multiplied by points_per_volt
        voltage_distance = self.voltage_distance_between_points(initial_point,final_point)
        num_steps = int(np.ceil(voltage_distance * points_per_volt))

        # Generate voltage arrays for a smooth, sinusoidally-spaced trajectory between the initial and final voltages
        smooth_factors = ( 1.0 - np.cos ( np.linspace(0.0, np.pi, num_steps) ) ) / ( 2.0 )
        displacements = np.outer(smooth_factors, final_volts-initial_volts)
        initial_volts_array = np.outer(np.ones(num_steps), initial_volts)

        total_array = displacements+initial_volts_array

        # Add additional row at end
        return np.vstack([total_array, total_array[-1]])
    
    def linear_voltages(self, initial_point, final_point, num_pixels, avgs_per_pixel):
        '''
        Generate a numpy array of voltages corresponding to a linspace path from initial_point to final_point.

        Arguments:
                *initial_point: dictionary containig axis names mapped to taret values (in um), e.g. {'x': 0.5, 'y':1.5}
                *final_point:   dictionary containig axis names mapped to taret values (in um), e.g. {'x': 0.5, 'y':1.5}
                *num_pixels:    number of unique voltage steps in the returned array
                *avgs_per_pixel:number of times each voltage step is repeated

        Return:
                Numpy array of voltages corresponding to a linspace path from initial_point to final_point; the last point is duplicated
        '''
        # Get rid of netrefs
        initial_point = obtain(initial_point)
        final_point = obtain(final_point)
        num_pixels = obtain(num_pixels)
        avgs_per_pixel = obtain(avgs_per_pixel)

        # Reformat point dictionaries and convert um to V
        initial_volts = np.array([self.um_to_V(initial_point['x'],'x'), self.um_to_V(initial_point['y'],'y')])
        final_volts   = np.array([self.um_to_V(final_point['x'],'x'),  self.um_to_V(final_point['y']  ,'y')])

        x_voltage_array = np.linspace(initial_volts[0], final_volts[0], num_pixels)
        y_voltage_array = np.linspace(initial_volts[1], final_volts[1], num_pixels)

        base_array = np.array([x_voltage_array,y_voltage_array]).transpose()

        # Repeat each unique row avgs_per_pixel number of times
        repeated_array = np.repeat(base_array, avgs_per_pixel, axis = 0)

        # Add additional row at end
        return np.vstack([repeated_array, repeated_array[-1]])

    def new_ctr_task(self, counter_channel, data_channel):
        '''
        Creates a new counter task on a specified channel (counter channel).
        
        Arguments:
                *counter_channeel: DAQ counter channel, should be something like 'Dev1/ctr0'
                *data_channel: DAQ data channel for collecting digital pulses, e.g. 'Dev1/PFI1'
        '''

        
        if data_channel[0] != '/':
            data_channel = '/' + data_channel

        self.ctr_task = nidaqmx.Task()
        self.ctr_task.ci_channels.add_ci_count_edges_chan(counter_channel)
        self.ctr_task.ci_channels.all.ci_count_edges_term = data_channel
        self.ctrTasks.append(self.ctr_task)


    def get_x_channel(self):
        return self.x_axis

    def get_y_channel(self):
        return self.y_axis
    
    def get_ctr_channel(self):
        return self.ctr_ch
    
    def get_XperV(self):
        return self.XperV
    
    def get_YperV(self):
        return self.YperV

    def get_axes(self):
        return self.axesDict

    def get_position(self):
        return self.position
    
    def set_x_channel(self, channel):
        self.x_axis = channel
    
    def set_y_channel(self, channel):
        self.y_axis = channel

    def set_ctr_channel(self, channel):
        self.ctr_ch = channel

    def set_XperV(self, cal):
        self.XperV = cal

    def set_YperV(self, cal):
        self.YperV = cal

    def set_position(self, point):
        self.position = point
    
    
if __name__=='__main__':
    '''
    # Test 1: See voltages move back and forth between two points
    with nidaqMotionControl(x_ch= 'Dev1/ao0', y_ch='Dev1/ao1', ctr_ch='Dev1/ctr0', XperV = 1, YperV = 1) as daq:
        # with these settings, it should be simple to find this waveform on a scope. Use 1ms/div for horizontal axis and 1V/div for vertical axis
        wait_time = 0.001 #s
        cycles = 100
        for i in range(cycles):
            daq.move({'x': -1, 'y': 3}, 50)
            time.sleep(wait_time)
            daq.move({'x': 0,   'y':0}, 50)
            time.sleep(wait_time)
    '''
    '''
    # Test 2: Move relative
    with nidaqMotionControl(x_ch= 'Dev1/ao0', y_ch='Dev1/ao1', ctr_ch='Dev1/ctr0', XperV = 1, YperV = 1) as daq:
        # with these settings, it should be simple to find this waveform on a scope. Use 1ms/div for horizontal axis and 1V/div for vertical axis
        wait_time = 0.5 #s
        cycles = 100
        for i in range(cycles):
            print('Cycle # ' + str(i))
            daq.move_relative({'x': -1, 'y': 1})
            time.sleep(wait_time)
            daq.move_relative({'x':  1, 'y':-1})
            time.sleep(wait_time)
    '''
    '''
    # Test 3: See a slow 1D scan
    with nidaqMotionControl(x_ch = 'Dev1/ao0', y_ch = 'Dev1/ao1', ctr_ch='Dev1/ctr0', XperV = 1, YperV = 1) as daq:
        wait_time = 0.1
        cycles = 2
        for i in range(cycles):
            print('Cycle # ' + str(i))
            #counts = daq.oneD_scan(initial_point={'x': -2, 'y': 0.5}, final_point= {'x': 0.5, 'y': 1.5}, num_pixels = 10, scan_rate = 20,   avgs_per_pixel = 5, data_channel = 'Dev1/PFI1') # small scan
            counts = daq.oneD_scan(initial_point={'x': -2, 'y': -10}, final_point= {'x': -2, 'y': 10},  num_pixels = 40, scan_rate = 1,  avgs_per_pixel = 20, data_channel = 'Dev1/PFI1') # big scan
            print('Cps' + str(counts))
            time.sleep(wait_time)
    '''
    '''
    # Test 4: See a slow 2D scna
    with nidaqMotionControl(x_ch = 'Dev1/ao0', y_ch = 'Dev1/ao1', ctr_ch='Dev1/ctr0', XperV = 5e-6, YperV = 5e-6) as daq:
        wait_time = 0.1
        cycles = 2
        for i in range(cycles):
            print('Cycle # ' + str(i))
            rates = daq.twoD_scan(initial_point= {'x': -50e-6, 'y': 50e-6}, final_point= {'x': 50e-6, 'y': -50e-6}, num_pixels_x = 20, num_pixels_y= 20, scan_rate = 100,  avgs_per_pixel = 5, data_channel = 'Dev1/PFI1')
            print('Cps')
            print(rates)
            time.sleep(wait_time)
    '''
    
    # Test 5: Test move with current position
    with nidaqMotionControl(x_ch = 'Dev1/ao0', y_ch = 'Dev1/ao1', ctr_ch='Dev1/ctr0', XperV = 5e-6, YperV = 5e-6) as daq:
        current_position = daq.read_current_position()
        print(current_position)

        daq.move({'x':-20e-6,'y':25e-6},10)

        current_position = daq.read_current_position()
        print(current_position)

        print(daq.get_position())
    