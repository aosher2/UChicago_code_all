"""
    Generalized NI DAQ class that exposes some basic functionality for general-purpose use

    Author: Benjamin Soloway, Jacob Feder
    Date: 5/13/2021
"""

# std
from contextlib import ExitStack
from nidaqmx.constants import AcquisitionType

# 3rd party
import numpy as np
import nidaqmx
from nidaqmx.stream_readers import CounterReader
from nidaqmx.constants import (Edge, TriggerType)

# lantz
from lantz import Driver

from nspyre.definitions import Q_ #TODO: change this to a lantz import

class NIDAQ(Driver):
    def __init__(self):
        pass

    def read_ctrs_single(self, ctr_channels, period):
        """Read the number of pulses received in a given sampling period using a hardware-timed clock
        ctr_channels: iterable of counter channels to use, e.g. or ['Dev1/ctr1'] or ['Dev1/ctr1', 'Dev1/ctr2']
        period: time to read (as a quantity object)
        return: list of (number of counts per second) for each counter channel
        """
        return [singleton[0] for singleton in self.read_ctrs_many(ctr_channels, period, 1)]

    def read_ctrs_many(self, ctr_channels, period, num_samples):
        """Repeatedly read the number of pulses received in a given sampling period using a hardware-timed clock
        ctr_channels: iterable of counter channels to use, e.g. or ['Dev1/ctr1'] or ['Dev1/ctr1', 'Dev1/ctr2']
        period: time to read for each sample before moving onto the next sample (as a quantity object)
        num_samples: number of samples
        return: list of (np arrays of counts per second corresponding to each sample) for each counter channel
        """
        # e.g. "Dev1"
        device_name = ctr_channels[0].split('/')[0]
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
            # create a digital input dummy task to start the di/SampleClock for clocking the counter input task
            sample_clk_task.di_channels.add_di_chan('{0}/port0'.format(device_name))
            sample_clk_task.timing.cfg_samp_clk_timing(acq_rate.to('Hz').m, samps_per_chan=num_samples)

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
                    acq_rate.to('Hz').m,
                    source='/{0}/di/SampleClock'.format(device_name),
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
                sample_reader_stream.read_many_sample_uint32(this_counter_raw_counts,
                                                number_of_samples_per_channel=nidaqmx.constants.READ_ALL_AVAILABLE,
                                                timeout=num_samples*period.to('s').m + 1)
                # calculate the difference in counts between each period
                counts = np.diff(this_counter_raw_counts) * acq_rate.to('Hz').m
                all_counts.append(counts)

        return all_counts

    def read_ctrs_ext_clk(self, ctr_channels, num_samples, acq_rate=Q_(100, 'MHz')):
        """Repeatedly read the number of pulses received in a given sampling period using a hardware-timed clock
        ctr_channels: iterable of counter channels to use, e.g. or ['Dev1/ctr1'] or ['Dev1/ctr1', 'Dev1/ctr2']
        period: time to read for each sample before moving onto the next sample (as a quantity object)
        num_samples: number of samples
        return: list of (np arrays of counts corresponding to each sample) for each counter channel
        """

        device_name = ctr_channels[0].split('/')[0] # e.g. "Dev1"
        clk_channel = '/{0}/PFI0'.format(device_name)
        trig_channel = '/{0}/PFI8'.format(device_name)

        # list of np arrays containing raw counts for each counter
        # e.g. 3x ctr_channels with num_samples = 5:
        # [ np.array([5, 10, 20, 30, 35]), np.array([7, 7, 10, 11, 12]), np.array([0, 0, 2, 4, 5]) ]
        all_counts = []
        # create DAQ tasks
        with ExitStack() as stack:
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
                    acq_rate.to('Hz').m,
                    source=clk_channel,
                    active_edge=nidaqmx.constants.Edge.RISING,
                    sample_mode=nidaqmx.constants.AcquisitionType.FINITE,
                    samps_per_chan=num_samples
                )

                #create trigger for reading to begin at the beginning of the sequence
                counter_task.triggers.arm_start_trigger.trig_type = TriggerType.DIGITAL_EDGE
                counter_task.triggers.arm_start_trigger.dig_edge_edge = Edge.RISING
                counter_task.triggers.arm_start_trigger.dig_edge_src = trig_channel

                # create counter input stream object
                sample_reader_stream = CounterReader(counter_task.in_stream)
                reader_streams.append(sample_reader_stream)
                # start the task
                counter_task.start()

            for sample_reader_stream in reader_streams:
                this_counter_raw_counts = np.zeros(num_samples, dtype=np.uint32)
                # read the counts out of the buffer
                # TODO check for timeout
                sample_reader_stream.read_many_sample_uint32(this_counter_raw_counts,
                                                number_of_samples_per_channel=nidaqmx.constants.READ_ALL_AVAILABLE,
                                                timeout=num_samples*(1/acq_rate).to('s').m + 1)
                # calculate the difference in counts between each period
                all_counts.append(this_counter_raw_counts)

        return all_counts