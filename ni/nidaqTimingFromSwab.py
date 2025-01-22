''' 

NI-DAQ Driver to count photons using Swab-Clocking

Chris Egerstrom, Nov 2023

Adapted from Sanskriti Chitransh, Evan Villafranca, Jacob Feder

Note: this DAQ doesn't live on the instrument server. 
The driver gets called whenever needed purely as a python file to communicate with the DAQ

'''



import numpy as np
import nidaqmx
from nidaqmx.stream_readers import CounterReader
from nidaqmx.constants import Edge, TriggerType, TaskMode, AcquisitionType, READ_ALL_AVAILABLE

from contextlib import ExitStack

class TreelessNIDAQ():

    CLK_CHANNEL = '/Dev1/PFI3'     # external clock from the Swabian
    APD_CHANNEL = '/Dev1/PFI0'     # TTL-ish clicks from the APD
    FLAG_CHANNEL = '/Dev1/PFI5'     # clicks to signal the start of experiment (or other need, but pulse seq-defined)

    def __init__(self):
        pass


    def __enter__(self):
        self.read_tasks = []
        self.reader_streams = []
        return self
    

    def __exit__(self, *args):  # execute when the DAQ object gets killed unexpectedly, for example, when the STOP button is clicked.
        if len(self.read_tasks) != 0:  # in case the DAQ object was killed before the reading was over, close and destroy all read tasks
            for read_task, reader_stream in zip(self.read_tasks, self.reader_streams):
                # print('in if')
                read_task.stop()
                read_task.close()
                self.read_tasks.remove(read_task)
                self.reader_streams.remove(reader_stream)


    def start_read_tasks_swabTimed(self, num_samples, trig_channel=FLAG_CHANNEL):     # create read task, set up counter, source clock and reader stream.

        # creating DAQ tasks  
        photonReadTask = nidaqmx.Task()
        self.read_tasks.append(photonReadTask)
        # adding digital input channel as counter (APD)
        photonReadTask.ci_channels.add_ci_count_edges_chan('/Dev1/ctr0')
        # connecting physical input to virtual counter
        photonReadTask.ci_channels.all.ci_count_edges_term = self.APD_CHANNEL
        # setting up timing clock (external)
        photonReadTask.timing.cfg_samp_clk_timing(
                                20e6,   # max DAQ sampling rate for convenience
                                source = self.CLK_CHANNEL, # Swabian clock ticks
                                active_edge=nidaqmx.constants.Edge.RISING,
                                sample_mode = nidaqmx.constants.AcquisitionType.FINITE,
                                samps_per_chan = num_samples, # number of ticks to be collected
        )
        photonReadTask.triggers.arm_start_trigger.trig_type = TriggerType.DIGITAL_EDGE
        photonReadTask.triggers.arm_start_trigger.dig_edge_edge = Edge.RISING
        photonReadTask.triggers.arm_start_trigger.dig_edge_src = trig_channel
        photonReadTask.control(TaskMode.TASK_COMMIT) #helps to initialize all tasks at once

        #Do this all again for the tracking/flag channel
        trackingTask = nidaqmx.Task()
        self.read_tasks.append(trackingTask)

        trackingTask.ci_channels.add_ci_count_edges_chan('/Dev1/ctr1')
        trackingTask.ci_channels.all.ci_count_edges_term = self.FLAG_CHANNEL
        trackingTask.timing.cfg_samp_clk_timing(
                        20e6,
                        source=self.CLK_CHANNEL,
                        active_edge=nidaqmx.constants.Edge.RISING,
                        sample_mode=nidaqmx.constants.AcquisitionType.FINITE,
                        samps_per_chan=num_samples,
                    )
        trackingTask.triggers.arm_start_trigger.trig_type = TriggerType.DIGITAL_EDGE
        trackingTask.triggers.arm_start_trigger.dig_edge_edge = Edge.RISING
        trackingTask.triggers.arm_start_trigger.dig_edge_src = trig_channel
        trackingTask.control(TaskMode.TASK_COMMIT)

        # creating counter stream object for counting
        self.reader_streams = [nidaqmx.stream_readers.CounterReader(photonReadTask.in_stream),
                               nidaqmx.stream_readers.CounterReader(trackingTask.in_stream)]
        # starting counting task
        photonReadTask.start()
        trackingTask.start()


    def read_samples(self, num_samples): 

        # reading out the counters
        photonCts = np.zeros(num_samples, dtype=np.uint32)
        trackingCts = np.zeros(num_samples, dtype=np.uint32)
        # read the counts out of the buffer
        self.reader_streams[0].read_many_sample_uint32(photonCts,
                                    number_of_samples_per_channel=nidaqmx.constants.READ_ALL_AVAILABLE,
                                    timeout=nidaqmx.constants.WAIT_INFINITELY)
        self.reader_streams[1].read_many_sample_uint32(trackingCts,
                                    number_of_samples_per_channel=nidaqmx.constants.READ_ALL_AVAILABLE,
                                    timeout=nidaqmx.constants.WAIT_INFINITELY)
        

        for read_task, reader_stream in zip(self.read_tasks, self.reader_streams):
            read_task.stop()
            read_task.close()
            self.read_tasks.remove(read_task)
            self.reader_streams.remove(reader_stream)

        # calculate difference in counts between consecutive clock ticks
        counts = np.diff(photonCts)
        flags = np.diff(trackingCts)

        return counts, flags


    def readCtr_multiRead_intClk(self, acqRate, num_samples:int, ctrChan='/Dev1/PFI0'): #only kept from legacy for TvT
        '''Reads specified counter channels for a given acqRate, a designated number of times (based off internal timing)'''

        num_samples += 1 #so np.diff works later. Also, a DAQ buffer size of 1 isn't supported

        #create DAQ tasks
        with nidaqmx.Task() as dummyClkTask, nidaqmx.Task() as photonReadTask:
            # create a digital input dummy task to start the di/SampleClock@acqRate for clocking the counter input task
            dummyClkTask.di_channels.add_di_chan('Dev1/port0')
            dummyClkTask.timing.cfg_samp_clk_timing(acqRate, sample_mode=AcquisitionType.CONTINUOUS)
            dummyClkTask.control(TaskMode.TASK_COMMIT)

            # adding digital input channel as counter (APD)
            photonReadTask.ci_channels.add_ci_count_edges_chan('/Dev1/ctr0')
            # connecting physical input to virtual counter
            photonReadTask.ci_channels.all.ci_count_edges_term = ctrChan
            # setting up timing clock (external)
            photonReadTask.timing.cfg_samp_clk_timing(
                                    acqRate, 
                                    source = '/Dev1/di/SampleClock', #internal sample clock
                                    active_edge=nidaqmx.constants.Edge.RISING,
                                    sample_mode = nidaqmx.constants.AcquisitionType.FINITE,
                                    samps_per_chan = num_samples, # number of ticks to be collected
            )

            #connect buffer to a reader
            photonReader = nidaqmx.stream_readers.CounterReader(photonReadTask.in_stream)
            #start the counter task
            photonReadTask.start()
            #then the timer it uses
            dummyClkTask.start()
            
            #set up struct to take data
            ctrRawCts = np.zeros(num_samples, dtype=np.uint32)
            # read the counts out of the buffer
            photonReader.read_many_sample_uint32(ctrRawCts,
                                            number_of_samples_per_channel=nidaqmx.constants.READ_ALL_AVAILABLE,
                                            timeout=num_samples/acqRate + 1) #1s overhead should be fine
                
        return np.diff(ctrRawCts)
    

    def readCtr_singleRead_intClk(self, acqRate, ctrChan='/Dev1/PFI0'): #only kept from legacy for TvT
        return(self.readCtr_multiRead_intClk(acqRate, 1, ctrChan))


    


   
    


   

        


    
