
"""
    drivers.ni.ni_motion_controller
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Generalized NI DAQ-based motion controller that uses analog outputs to drive
    motion stages.

    Author: Uri Zvi, Jacob Feder, Aidan Jones
    Date: 9/30/2020

    Modifications: Nazar, Dec 2020, cleaned; 
                   Chris, Dec 2022, de-lantz-ed and simplified by assuming all voltages in V, pos/dist's in um, and acq_rates in Hz. TODO: VERIFY I DIDN'T BREAK ANYTHING
                   
    WARNING: THE FSM DRIVER IS DEPENDENT ON THIS FILE
"""

# std lib
from collections import OrderedDict
import time

# 3rd party
import numpy as np
import nidaqmx
from nidaqmx.stream_writers import AnalogMultiChannelWriter
from nidaqmx.stream_readers import CounterReader

#custom
#from drivers.ni.nidaqTimingFromSwab import TreelessNIDAQ

class NIDAQAxis():
    DEFAULT_UNITS_DISTANCE = 'um' #these are never used, just want them to be standard and conveniently accessible
    DEFAULT_UNITS_RATE = 'Hz'
    DEFAULT_UNITS_VOLTAGE = 'V'


    def __init__(self, ao_ch, cal, limits=(None, None)):#, units='um'):
        """Class representing an individual axis associated with an individual 
        DAQ analog output (ao) channel. **New: Distances assumed to be in um**
        Arguments:  *ao_ch: NI DAQ channel string e.g. 'Dev/ao0'
                    *cal: Calibration in V/um
                    *limits: Limits tuple in um e.g. (0, 1e5) for 0 to 10cm. Default: (None,None)
        """
        if not (isinstance(limits[0], float) or isinstance(limits[0], int) or (limits[0] is None)) or not (isinstance(cal, float) or isinstance(cal, int)): #Might not be necessary, just good 
            raise TypeError('New Driver assumes distances in um and voltages in V. Please un-Pint your input by converting manually and removing units.') #for enforcing version differences now
        self.ch = ao_ch
        self.limits = limits
        self.cal = cal


    def um_to_volts(self, pos):
        """Convert a given position (in um) to a voltage (in V)
        Arguments:  *pos: position in um to convert
        Return: corresponding voltage (in V)"""
        return (pos * self.cal)



class NIDAQMotionController():
    def __init__(self, ctr_ch, acq_rate, axes, ao_smooth_steps=5000):
        """Motion controller for an n-dimensional set of NI DAQ analog output axes
        Arguments:  *ctr_ch: string for DAQ name e.g. 'Dev1'
                    *acq_rate: acquire rate for DAQ (in Hz)
                    *axes: dictionary of axis name mapped to NIDAQAxis object
                    *ao_smooth_steps: # of steps to take per volt change. Default: 5000 

        Example Initialization:
        axis1 = NIDAQAxis('Dev1/ao0', cal0_in_V/m) #With optional limits
        axis2 = NIDAQAxis('Dev1/ao1', cal1_in_V/m)
        mc = NIDAQMotionController('Dev1', {'x' : axis1, 'y' : axis2})
        or
        mc = NIDAQMotionController('Dev1', x=axis1, y=axis2)
        """
        self.axes = OrderedDict(axes)
        self.position = {}
        for a in self.axes:
            self.position[a] = 0
        self.acq_rate = acq_rate
        self.ctr_ch = ctr_ch
        self.ao_smooth_steps = ao_smooth_steps
        self.current_counter_task = None
        self.counter_tasks = []


    def __enter__(self):
        '''Actually initialize and connect to device'''
        self.ao_motion_task = nidaqmx.Task('NIDAQMotionController_AO')
        for a in self.axes:
            min_max_dict = {}
            # TODO allow limits specified in volts as well
            if self.axes[a].limits[0]:
                min_max_dict['min_val'] = self.axes[a].um_to_volts(self.axes[a].limits[0])
            if self.axes[a].limits[1]:
                min_max_dict['max_val'] = self.axes[a].um_to_volts(self.axes[a].limits[1])
            self.ao_motion_task.ao_channels.add_ao_voltage_chan(self.axes[a].ch, 
                                    name_to_assign_to_channel=a,
                                    **min_max_dict)
        return(self)


    def __exit__(self, *args):
        '''Closes out motion and all counter tasks'''
        self.ao_motion_task.close()
        for t in self.counter_tasks:
            t.close()


    def new_ctr_task(self, ctr_ch):
        '''Creates a new counter task on a specified ctr_ch
        Arguments:  *ctr_ch: new DAQ counter channel e.g. 'Dev1' '''
        self.current_counter_task = nidaqmx.Task('NIDAQMotionController_CTR_{}'.format(np.random.randint(2**31))) #WHAT IN THE SPAGHETTI? -CE. Probably only have a problem if you're really unlucky -Also CE
        self.current_counter_task.ci_channels.add_ci_count_edges_chan(ctr_ch)
        self.current_counter_task.ci_channels.all.ci_count_edges_term = '/Dev1/PFI0'#TreelessNIDAQ.APD_CHANNEL #Hard-coded to take all counts channel from Jasper #TODO: Un-hardcode this?
        self.counter_tasks.append(self.current_counter_task)


    def move(self, point):
        """Move to a target location using a smooth s-curve-like interpolation
        Arguemnts:  *point: dictionary containing axis names mapped to target values (in um) e.g. {'x': 0.5, 'y': 1.5}
        """
        #import pdb; pdb.set_trace()
        step_voltages = self.smooth_func(self.position, point)
        #print ('I am moving')
        #print(step_voltages)
        #print(type(step_voltages))
        if step_voltages.size:
            self.ao_motion_task.timing.cfg_samp_clk_timing(
                self.acq_rate,
                sample_mode=nidaqmx.constants.AcquisitionType.FINITE,
                samps_per_chan=step_voltages.shape[0]
            )
            #print("working on line 135")
            self.ao_motion_task.triggers.start_trigger.disable_start_trig()
            sample_writer_stream = AnalogMultiChannelWriter(self.ao_motion_task.out_stream,
                                                                auto_start=False)
            #print("working on line 139")
            # must use array with contiguous memory region because NI uses C arrays under the hood
            ni_sample_buffer = np.ascontiguousarray(step_voltages.transpose(), dtype=float)
            #print("buffer made")
            # TODO timeout
            sample_writer_stream.write_many_sample(ni_sample_buffer, timeout=10)
            #print("samples written")

            self.ao_motion_task.start()
            time.sleep(1.1*step_voltages.shape[0] / self.acq_rate)
            self.ao_motion_task.stop()
        #print("move() almost finishes")
        self.position = point


    def line_scan(self, init_point, final_point, steps, pts_per_step=100):
        """1-axis line scan while acquiring counter data
        Arguments:  *init_point: dict of axis names mapped to starting values (in um) e.g. {'x': 0.5, 'y': 1.5}
                    *final_point: dict of axis names mapped to final values (in um) e.g. {'x': 0.5, 'y': 1.5}
                    *steps: number of steps to take between init_ and final_point s
                    *pts_per_step: number of pts to take at each step. Default: 1
        Returns: Avg'd data normalized by acquire rate in a Numpy array""" #TODO: confirm pts_per_step and return info is correct
        #print(init_point)
        #print(final_point)
        
        #print(init_point)
        self.move(init_point)
        step_voltages = self.linear_func(init_point, final_point, steps)
        step_voltages = np.repeat(step_voltages, pts_per_step + 1, axis=0) # TODO: What the fuck is this? pts_per_step
        #print(step_voltages)
        # configure analog output task
        self.ao_motion_task.timing.cfg_samp_clk_timing(
            self.acq_rate,
            sample_mode=nidaqmx.constants.AcquisitionType.FINITE,
            samps_per_chan=step_voltages.shape[0]
        )
        #self.ao_motion_task.triggers.start_trigger.disable_start_trig()
        sample_writer_stream = AnalogMultiChannelWriter(self.ao_motion_task.out_stream,
                                                            auto_start=False)
        # must use array with contiguous memory region because NI uses C arrays under the hood
        ni_ao_sample_buffer = np.ascontiguousarray(step_voltages.transpose(), dtype=np.float64)
        size = len(ni_ao_sample_buffer)
        print(size)
        # TODO timeout
        sample_writer_stream.write_many_sample(ni_ao_sample_buffer, timeout=60)
        
        # e.g. "Dev1"
        device_name = 'Dev1' #list(self.axes.items())[0][1].ch.split('/')[0]
        
        #Create a new counter input task #DEBUG
        self.new_ctr_task(self.ctr_ch) #DEBUG
        # configure counter input task
        self.current_counter_task.timing.cfg_samp_clk_timing(
            self.acq_rate,
            source= "",#'/{}/ao/SampleClock'.format(device_name),
            sample_mode=nidaqmx.constants.AcquisitionType.FINITE,
            samps_per_chan=step_voltages.shape[0],
        )
        # set the counter input to trigger / start acquisition when the AO starts
        self.current_counter_task.triggers.arm_start_trigger.dig_edge_src = '/{}/ao/StartTrigger'.format(device_name)
        self.current_counter_task.triggers.arm_start_trigger.trig_type = nidaqmx.constants.TriggerType.DIGITAL_EDGE
        # create counter stream object
        sample_reader_stream = CounterReader(self.current_counter_task.in_stream)
        # must use array with contiguous memory region because NI uses C arrays under the hood
        ni_ctr_sample_buffer = np.ascontiguousarray(np.zeros(step_voltages.shape[0]), dtype=np.uint32)
        
        # start the move
        self.current_counter_task.start()
        print("Started counting")
        self.ao_motion_task.start()
        print("Started ao_motion")
        # TODO timeout
        sample_reader_stream.read_many_sample_uint32(ni_ctr_sample_buffer,
                                        number_of_samples_per_channel=nidaqmx.constants.READ_ALL_AVAILABLE,
                                        timeout=60)
        
        # wait for motion to complete
        #time.sleep((1.1*step_voltages.shape[0] / self.acq_rate).to('s').m)
        
        self.current_counter_task.stop()
        self.ao_motion_task.stop()

        #New Clean-up to hopefully prevent DAQ errors
        self.current_counter_task.close()
        self.counter_tasks.remove(self.current_counter_task)
        self.current_counter_task = None

        scanned = ni_ctr_sample_buffer.reshape((steps, pts_per_step+1))
        averaged = np.diff(scanned).mean(axis=1)
        self.position = final_point
        #self.__exit__() #DEBUG #total jank, but show free up all resources to run things again
        #self.__enter__()
        return averaged*self.acq_rate #averaged


    def linear_func(self, start_pt, stop_pt, steps):
        """Generate a set of linearly spaced points between specified start and stop points
        Arguments:  *start_pt: dict of axis names mapped to starting values (in um) e.g. {'x': 0.5, 'y': 1.5}
                    *stop_pt: dict of axis names mapped to final values (in um) e.g. {'x': 0.5, 'y': 1.5}
                    *steps: number of steps to take between start_ and stop_point s
        Return: steps-long numpy array of voltages corresponding to linear path from start_pt to stop_pt"""
        start_volts = np.array([])
        stop_volts = np.array([])
        for axis in self.axes:
            start_volts = np.append(start_volts, self.axes[axis].um_to_volts(start_pt[axis]))
            stop_volts = np.append(stop_volts, self.axes[axis].um_to_volts(stop_pt[axis]))
        linear_steps = np.linspace(0.0, 1.0, steps)
        # pt0 = [0, 0]
        # pt1 =  [1, 2]
        # np.ones = [1, 1, 1, 1, 1, 1, ...]
                        # [[x, x, x, x, x, x....]
                        # [y, y, y, y, y, y, y...]]
                        
                        # [[0.1, 0.2, ..., 1]
                        # [0.2, 0.4, ..., 2]]
        return np.outer(np.ones(steps), start_volts) + np.outer(linear_steps, stop_volts-start_volts)


    def smooth_func(self, start_pt, stop_pt):
        """Generate a set of smooth, sinusoidally spaced points between the start and stop point
        Arguments:  *start_pt: dict of axis names mapped to starting values (in um) e.g. {'x': 0.5, 'y': 1.5}
                    *stop_pt: dict of axis names mapped to final values (in um) e.g. {'x': 0.5, 'y': 1.5}
        Return: Numpy array of voltages corresponding to smooth path from start_pt to stop_pt"""
        # convert start_pt and stop_pt dictionaries into arrays that contain
        # the corresponding point values in volts
        #import pdb; pdb.set_trace()
        start_volts = np.array([])
        stop_volts = np.array([])
        for axis in self.axes:
            start_volts = np.append(start_volts, self.axes[axis].um_to_volts(start_pt[axis]))
            stop_volts = np.append(stop_volts, self.axes[axis].um_to_volts(stop_pt[axis]))
        #print(start_volts, "are our start voltages")
        #print(stop_volts, "are our stop voltages")
        steps = int(np.ceil(max(abs(stop_volts - start_volts)) * self.ao_smooth_steps))
        # generate a cosine function from 0->pi
        versine_steps = (1.0 - np.cos(np.linspace(0.0, np.pi, steps))) / 2.0
        return np.outer(np.ones(steps), start_volts) + np.outer(versine_steps, stop_volts - start_volts)
