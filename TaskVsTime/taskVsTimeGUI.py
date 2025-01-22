"""
This is a GUI to run TaskVsTime on Jasper using new CustomGUI class

Copyright (c) September 2023, C. Egerstrom
All rights reserved.

This work is licensed under the terms of the 3-Clause BSD license.
For a copy, see <https://opensource.org/licenses/BSD-3-Clause>.
"""

import numpy as np
from nspyre import DataSink, FlexLinePlotWidget
from experiments.TaskVsTime import Automated_tvt
from experiments.General.generalGuiElements import CountsVsDVsExperimentWidget
from experiments.TaskVsTime import taskVsTimeExp


class TaskVsTimeWidget(CountsVsDVsExperimentWidget):
    
    def __init__(self):

        exp_params_config = {}

        super().__init__(defaultParamsDict={'defaultDatasetName': 'TvT', 'defaultSamplingFreq': 2, 
                        'defaultMaxIters': 7200, 'defaultAutosaveInterval': 1000000},
                         exp_params_config=exp_params_config,
                         module = Automated_tvt,
                         cls = 'TaskVsTimeMeasurement', 
                         fun_name='taskVsTime',
                         title='Task Vs Time',
                         )


def process_TvT_data(sink: DataSink):
    """Make Counts Datasets Numpy arrays in Python lists so FlexLinePlotWidget can handle them"""
    sink.datasets['CountsToPlot'] = [np.stack([sink.datasets['times'], np.array(sink.datasets['counts'])])]


class FlexLinePlotWidgetWithTVTDefaults(FlexLinePlotWidget):
    """Add some default settings to the FlexSinkLinePlotWidget."""
    def __init__(self):
        super().__init__(data_processing_func=process_TvT_data)
        # create some default signal plots
        self.add_plot('PFI0/Ctr0 (APD Counts)',        series='CountsToPlot',   scan_i='',     scan_j='',  processing='Average')

        # manually set the XY range
        #self.line_plot.plot_item().setXRange(3.0, 4.0)
        #self.line_plot.plot_item().setYRange(-3000, 4500)

        # retrieve legend object
        legend = self.line_plot.plot_widget.addLegend()
        # set the legend location
        legend.setOffset((-10, -50))

        self.datasource_lineedit.setText('TvT')