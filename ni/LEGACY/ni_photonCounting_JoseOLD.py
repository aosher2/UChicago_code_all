"""
    Generalized NI DAQ class that exposes some basic functionality for general-purpose use
    Author: Benjamin Soloway, Jacob Feder
    Date: 5/13/2021
"""

# std
from contextlib import ExitStack

# 3rd party
import numpy as np
import nidaqmx
from nidaqmx.stream_readers import CounterReader

class NIDAQ():
    def __init__(self):
        self.nothingness = 0

    def read_ctrs_single(self, ctr_channels, period):
        """Read the number of pulses received in a given sampling period using a hardware-timed clock
        Args:
            ctr_channels: iterable of counter channels to use, e.g. or ['Dev1/ctr1'] or ['Dev1/ctr1', 'Dev1/ctr2']
            period: time to read (as a quantity object)
            return: list of (number of counts per second) for each counter channel
        """
        return [singleton[0] for singleton in self.read_ctrs_many(ctr_channels, period, 1)]

    def read_ctrs_many(self, ctr_channels, period, num_samples):
        """Repeatedly read the number of pulses received in a given sampling period using a hardware-timed clock
        Args:
            ctr_channels: iterable of counter channels to use, e.g. or ['Dev1/ctr1'] or ['Dev1/ctr1', 'Dev1/ctr2']
            period: time to read for each sample before moving onto the next sample (s)
            num_samples: number of samples
            return: list of (np arrays of counts per second corresponding to each sample) for each counter channel
        """
        # e.g. "Dev1"
        device_name = ctr_channels[0].split('/')[0]
        # we will use a counter output to clock the counter input measurement
        # output counter channel to use for clocking the read
        sample_clk_channel = '{0}/ctr0'.format(device_name)
        sample_clk_terminal = '/{0}/Ctr0InternalOutput'.format(device_name)
        for ctr_channel in ctr_channels:
            if ctr_channel == sample_clk_channel:
                raise Exception(f'The specified counter channel ({ctr_channel}) is used to clock the NI DAQ task - choose a different channel')
        # clock rate of counter input (CI)
        acq_rate = 1 / period
        # increment the number of samples since we need to know the initial counter value
        # and will do np.diff later
        num_samples += 1

        # list of np arrays containing raw counts for each counter
        # e.g. 3x ctr_channels with num_samples = 5:
        # [ np.array([5, 10, 20, 30, 35]), np.array([7, 7, 10, 11, 12]), np.array([0, 0, 2, 4, 5]) ]
        all_counts = []

        # create DAQ tasks
        with nidaqmx.Task() as sample_clk_task, ExitStack() as stack:
            # create a counter that will generate a pulse train for clocking the counter input task 
            sample_clk_task.co_channels.add_co_pulse_chan_freq(sample_clk_channel, freq=acq_rate)
            sample_clk_task.timing.cfg_implicit_timing(samps_per_chan=num_samples)
            
            # array containing counter reader streams
            reader_streams = []
            # iterate through all the counter channels and perform setup
            for ctr_channel in ctr_channels:
                # this should automatically call the __enter__ and __exit__
                # methods for each task as if they were used in a 'with' statement
                counter_task = stack.enter_context(nidaqmx.Task())
                counter_task.ci_channels.add_ci_count_edges_chan(ctr_channel)
                # configure counter input task
                counter_task.timing.cfg_samp_clk_timing(
                    acq_rate,
                    source=sample_clk_terminal,
                    active_edge=nidaqmx.constants.Edge.RISING,
                    sample_mode=nidaqmx.constants.AcquisitionType.FINITE,
                    samps_per_chan=num_samples
                )
                # create counter input stream object
                sample_reader_stream = CounterReader(counter_task.in_stream)
                reader_streams.append(sample_reader_stream)
                # start the task
                counter_task.start()

            # start the data acquisition clock
            sample_clk_task.start()

            for sample_reader_stream in reader_streams:
                this_counter_raw_counts = np.zeros(num_samples, dtype=np.uint32)
                # read the counts out of the buffer
                # TODO check for timeout

                sample_reader_stream.read_many_sample_uint32(this_counter_raw_counts, number_of_samples_per_channel=nidaqmx.constants.READ_ALL_AVAILABLE,
                    timeout=num_samples*period + 1)
                
                # calculate the difference in counts between each period
                counts = np.diff(this_counter_raw_counts) * acq_rate
                all_counts.append(counts)

        return all_counts