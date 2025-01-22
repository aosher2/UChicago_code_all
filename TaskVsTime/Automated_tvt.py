"""
This is an experiment to run TaskVsTime on Jasper so that it can utilize FlexLinePlotWidget

Copyright (c) September 2023, C. Egerstrom
All rights reserved.

This work is licensed under the terms of the 3-Clause BSD license.
For a copy, see <https://opensource.org/licenses/BSD-3-Clause>.
"""

#TODO: LOGGING
import time 
#from rpyc.utils.classic import obtain
from nspyre import StreamingList, DataSource, experiment_widget_process_queue #, InstrumentGateway

from experiments.customUtils import setupIters
from drivers.ni.nidaqTimingFromSwab import TreelessNIDAQ


class TaskVsTimeMeasurement:

    def __init__(self, queue_to_exp, queue_from_exp):
        self.queue_to_exp=queue_to_exp
        self.queue_from_exp=queue_from_exp


    def taskVsTime(self, datasetName: str, maxIters: int, shouldAutosave:bool, autosaveInterval:int, debug=False, **kwargs):
        """Run a TaskVsTime experiment

        Args:
            datasetName: name of the dataset to push data to
            samplingFreq (float): how quickly to read data (in Hz)
            maxIters: max number of data points to collect. If negative, will go infinitely 
            shouldAutosave:bool, 
            autosaveInterval:int,
            debug: optional (default False), will run TimeVsTime if true
        """

        # connect to the data server and create a data set, or connect to an
        # existing one with the same name if it was created earlier.
        with TreelessNIDAQ() as nidaq, DataSource(datasetName) as tvt_data:
                    
            ctrChanNum = 'Dev1/PFI0'

            # for storing the experiment data
            self.times = StreamingList()
            self.counts = StreamingList()

            self.startTime = time.time() #take time diff measurements

            #setup a relevant iterator depending on maxIters
            #iters = setupIters(maxIters)
            iters = 8
            samplingFreq = 100 # 

            #MAIN EXPERIMENT LOOP
            for i in iters:

                if debug: #time vs time
                    self.counts.append(time.time())
                    time.sleep(1/samplingFreq) #need to set a delay somehow since 'read' is instant

                else: #normal operating mode
                    newData = nidaq.readCtr_singleRead_intClk(acqRate=samplingFreq, )
                    self.counts.append(newData[0])
                
                #Read and save new time
                self.times.append(time.time()-self.startTime)

                # save the current data to the data server.
                tvt_data.push({'params': {'DatasetName': datasetName, 'samplingFreq': samplingFreq, 'MaxIters': maxIters, 'CtrChanNum': ctrChanNum, 'DebugMode': debug, 'shouldAutosave':shouldAutosave, 'autosaveInterval':autosaveInterval, **kwargs},
                                'title': 'Task vs Time',
                                'xlabel': 'Time (s)',
                                'ylabel': 'Counts',
                                'datasets': {'times':self.times, 'counts': self.counts}
                })

                
                #nice closeout request from GUI (like Sweep Stop from LabView)
                if experiment_widget_process_queue(self.queue_to_exp) == 'stop':
                        # the GUI has asked us nicely to exit
                        return

                if shouldAutosave and (i+1)%autosaveInterval == 0: #Autosave logic, +1 so it doesn't autosave first data point
                    self.queue_from_exp.put_nowait(f'SAVE_REQ:AUTO of {datasetName} as TvT')



