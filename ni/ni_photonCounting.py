#import time

import numpy as np
import nidaqmx
from nidaqmx.stream_readers import CounterReader
from nidaqmx.constants import Edge, TriggerType, TaskMode, AcquisitionType, READ_ALL_AVAILABLE
from contextlib import ExitStack


class NIDAQ_PhotonCounter():
    def __init__(self):
        #APD channels named after their relevant switch values
        #Each switch # corresponds to the same #ed Swab Dig_Out
        self.rawAPDchannel = '/Dev1/PFI11'
        self.s1H_s2H_APDchannel = '/Dev1/PFI1'
        self.s1H_s2L_APDchannel = '/Dev1/PFI4'
        self.s1L_s3H_APDchannel = '/Dev1/PFI8'
        self.apdChannelsDict = {1: self.s1H_s2H_APDchannel, 4: self.s1H_s2L_APDchannel,
                                8: self.s1L_s3H_APDchannel, 11: self.rawAPDchannel }

       # self.trigChannel = '' #TODO: Add a trigger channel once it's connected
       # self.clockChannel = '' #TODO: add a clock channel to DAQ from Swab, and implement it in the extTrig ctr


    def __enter__(self):
        return(self)


    def readCtrs_multiRead_intClk(self, acqRate, numSamples:int, ctrChanNums=[11,1,4,8]):
        '''Reads specified counter channels for a given period, a designated number of times (based off internal timing)'''

        ctrChans = [self.apdChannelsDict[ctrChanNum] for ctrChanNum in ctrChanNums]
        
        period = 1/acqRate
        numSamples += 1 #so np.diff works later. Also, a DAQ buffer size of 1 isn't supported
        
        # list of np arrays containing raw counts for each counter
        # e.g. 3x ctr_channels with num_samples = 5:
        # [ np.array([5, 10, 20, 30, 35]), np.array([7, 7, 10, 11, 12]), np.array([0, 0, 2, 4, 5]) ]
        all_counts = []

        #create DAQ tasks
        with nidaqmx.Task() as dummyClkTask, ExitStack() as stack:
            # create a digital input dummy task to start the di/SampleClock@acqRate for clocking the counter input task
            dummyClkTask.di_channels.add_di_chan('Dev1/port0')
            dummyClkTask.timing.cfg_samp_clk_timing(acqRate, sample_mode=AcquisitionType.CONTINUOUS)
            dummyClkTask.control(TaskMode.TASK_COMMIT)

            # array containing counter reader streams
            readerStreams = []
            ctrTasks = []
            for i, ctrChan in enumerate(ctrChans):
                # this automatically calls the __enter__ and __exit__
                # methods for each task as if they were used in a 'with' statement
                ctrTask = stack.enter_context(nidaqmx.Task())
            
                #create a counter input task
                ctrTask.ci_channels.add_ci_count_edges_chan(f'Dev1/ctr{i}') #connect to a ctr
                ctrTask.ci_channels.all.ci_count_edges_term = ctrChan #connect counter to relevant PFI channel

                #configure the counter input task
                ctrTask.timing.cfg_samp_clk_timing(acqRate, source='/Dev1/di/SampleClock',samps_per_chan = numSamples)
                #source set this way since it uses dummyClkTask rate, and because onboard clock can't be used for ctr tasks
                #source='' (for default onboard clock), active_edge=nidaqmx.constants.Edge.RISING, sample_mode=nidaqmx.constants.AcquisitionType.FINITE, samps_per_chan=numSamples)
                
                #create counter input stream object for later
                readerStreams.append(CounterReader(ctrTask.in_stream))
                
                #Load up tasks to be run quickly together later. Could also just ctrTask.start(), 
                #but worried about more substantial desyncing issues than just starting everything in a python for loop
                ctrTask.control(TaskMode.TASK_COMMIT) #TODO: Get a 2nd opinion about this from Michael
                ctrTasks.append(ctrTask)

            #start the counter tasks
            for ctrTask in ctrTasks:
                ctrTask.start()
            #then the timer they use
            dummyClkTask.start()
            
            #read counters out
            for readerStream in readerStreams:
                thisCtrRawCts = np.zeros(numSamples, dtype=np.uint32)

                # read the counts out of the buffer
                readerStream.read_many_sample_uint32(thisCtrRawCts,
                                                number_of_samples_per_channel=nidaqmx.constants.READ_ALL_AVAILABLE,
                                                timeout=numSamples*period + 1) #1s overhead should be fine
                
                # calculate the difference in counts between each period
                all_counts.append(np.diff(thisCtrRawCts))
        #print(all_counts) #DEBUG
        return np.array(all_counts)
    

    def readCtrs_singleRead_intClk(self, acqRate, ctrChanNums=[11,1,4,8]):
        return(self.readCtrs_multiRead_intClk(acqRate, 1, ctrChanNums,))
    
    """
    def readCtrs_multiRead_extTrigAndClk_basedOnBen(self, ctrChanNums=[1,4,8,11], numSamples=5, acqRate = 100e6):
        '''Reads specified counter channels a designated number of times based off external timing'''
        
        ctrChans = [self.apdChannelsDict[ctrChanNum] for ctrChanNum in ctrChanNums]

        # list of np arrays containing raw counts for each counter
        # e.g. 3x ctr_channels with num_samples = 5:
        # [ np.array([5, 10, 20, 30, 35]), np.array([7, 7, 10, 11, 12]), np.array([0, 0, 2, 4, 5]) ]
        all_counts = []

        # create DAQ tasks
        readerStreams = []
        for ctrChan in ctrChans:
            with nidaqmx.Task() as ctrTask:
                #create a counter input task
                ctrTask.ci_channels.add_ci_count_edges_chan(ctrChan)

                #configure the counter input task
                ctrTask.timing.cfg_samp_clk_timing(acqRate, samps_per_chan = numSamples)
                #source='' (for default onboard clock), active_edge=nidaqmx.constants.Edge.RISING, sample_mode=nidaqmx.constants.AcquisitionType.FINITE, samps_per_chan=numSamples)
                #TODO: source should be changed to clk_channel once we have one
                
                #create trigger for reading to begin at the beginning of the sequence
                ctrTask.triggers.arm_start_trigger.trig_type = TriggerType.DIGITAL_EDGE
                ctrTask.triggers.arm_start_trigger.dig_edge_edge = Edge.RISING
                ctrTask.triggers.arm_start_trigger.dig_edge_src = self.trigChannel
                #TODO: Look into setting read windows up as watchdog tasks that can be killed with some sort of expiration trigger

                #create counter input stream object for later
                readerStreams.append(CounterReader(ctrTask.in_stream))
                
                #start the counter task
                ctrTask.start()
            

            for readerStream in readerStreams:
                thisCtrRawCts = np.zeros(numSamples, dtype=np.uint32)

                # read the counts out of the buffer
                readerStream.read_many_sample_uint32(thisCtrRawCts,
                                                number_of_samples_per_channel=nidaqmx.constants.READ_ALL_AVAILABLE,
                                                timeout=numSamples*(1/acqRate) + 1) #1s overhead should be fine
                
                # calculate the difference in counts between each period
                all_counts.append(np.append(np.zeros(1), thisCtrRawCts)) #add a leading 0 so diff works right

        return all_counts"""

    """def readCtrs_multiRead_extTrig_MoreMine(self, ctrChanNums=[1,4,8,11], numSamples=5, acqRate = 100e6):
        '''Reads specified counter channels a designated number of times based off external timing'''
        
        ctrChans = [self.apdChannelsDict[ctrChanNum] for ctrChanNum in ctrChanNums]

        # list of np arrays containing raw counts for each counter
        # e.g. 3x ctr_channels with num_samples = 5:
        # [ np.array([5, 10, 20, 30, 35]), np.array([7, 7, 10, 11, 12]), np.array([0, 0, 2, 4, 5]) ]
        all_counts = []

        # create DAQ tasks
        readerStreams = []
        for ctrChan in ctrChans:
            with nidaqmx.Task() as ctrTask:
                #create a counter input task
                ctrTask.ci_channels.add_ci_count_edges_chan(ctrChan)

                #configure the counter input task
                ctrTask.timing.cfg_samp_clk_timing(acqRate, samps_per_chan = numSamples)
                #source='' (for default onboard clock), active_edge=nidaqmx.constants.Edge.RISING, sample_mode=nidaqmx.constants.AcquisitionType.FINITE, samps_per_chan=numSamples)
                
                #create trigger for reading to begin at the beginning of the sequence
                ctrTask.triggers.arm_start_trigger.trig_type = TriggerType.DIGITAL_EDGE
                ctrTask.triggers.arm_start_trigger.dig_edge_edge = Edge.RISING
                ctrTask.triggers.arm_start_trigger.dig_edge_src = self.trigChannel 
                #TODO: Look into setting read windows up as watchdog tasks that can be killed with some sort of expiration trigger

                #create counter input stream object for later
                readerStreams.append(CounterReader(ctrTask.in_stream))
                
                #start the counter task
                ctrTask.start()"""
    
if __name__ == '__main__':
    daq = NIDAQ_PhotonCounter()
    print(daq.readCtrs_singleRead_intClk(acqRate=1)) #1s-long readout
