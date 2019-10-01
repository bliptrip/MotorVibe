#!/usr/bin/env python3
# Author: Andrew Maule
# Purpose: This code generates an output html file with spectral density plots of motor vibrations.
#   NOTE: Shamelessly stolen from PX4's flight review plot code @ https://github.com/px4/flight_review,
#   specifically under plot_app/plotting.py.
#   

from argparse import ArgumentParser
from bokeh.layouts import gridplot
from bokeh.plotting import figure, output_file, show
#pylint: disable=line-too-long, arguments-differ, unused-import
from bokeh.models import (
    ColumnDataSource, Range1d, DataRange1d, FuncTickFormatter, Legend, Span, 
    BoxZoomTool, PanTool, WheelZoomTool, ResetTool, SaveTool,
    LabelSet, ColorBar, LinearColorMapper, BasicTicker, PrintfTickFormatter
    )
from bokeh.palettes import viridis
import numpy as np
from pathlib import Path
import pyfftw #Uses fftw library, as it is faster than scipy's fft library
from pyulog import *
from pyulog.px4 import *
import re
import scipy
import scipy.signal
import traceback

# general configuration variables for plotting
plot_width = 840

plot_color_blue = '#2877a2' # or: #3539e0

plot_config = dict(
    maps_line_color = plot_color_blue,
    plot_width = plot_width,
    plot_height = dict(
        normal = int(plot_width / 2.1),
        small = int(plot_width / 2.5),
        large = int(plot_width / 1.61803398874989484), # used for the gps map
        ),
    )

TOOLS = "pan,wheel_zoom,box_zoom,reset,save"
ACTIVE_SCROLL_TOOLS = "wheel_zoom"

def parse_args():
    parser = ArgumentParser(description='Generate spectral density plots from pyulog data files.')
    parser.add_argument('-o', '--output', dest='output', required=True, help='Output html file to dump plots to.')
    parser.add_argument('-m', '--map', dest='map',
            help="Dictionary containing a map of first motor's pwm to another equivalent pwm (to compare approximately the same RPM value)",
            default="{1300000:1163158,1400000:1252632,1500000:1342105,1600000:1431579,1700000:1521053,1800000:1610526,1900000:1700000}")
    parser.add_argument('--motor1', dest='motor1', help='Name of motor 1', default='sunnysky')
    parser.add_argument('--motor2', dest='motor2', help='Name of motor 2.', default='tmotor')
    parser.add_argument('-r', '--regexp', dest='regexp', help='Regular expression to use for matching correct paths to search for PX4 ulog files.', default=r'^((tmotor)|(sunnysky)).+')
    parser.add_argument('-i', '--ignore', dest='ignore', action='store_true',
                        help='Ignore string parsing exceptions', default=True)
    parsed = parser.parse_args()
    return(parsed)

class DataPlot:
    """
    Handle the bokeh plot generation from an ULog dataset
    """


    def __init__(self, data, config, data_name, x_axis_label=None,
                 y_axis_label=None, title=None, plot_height='normal',
                 y_range=None, y_start=None, changed_params=None,
                 topic_instance=0, x_range=None):

        self._had_error = False
        self._previous_success = False
        self._param_change_label = None

        self._data = data
        self._config = config
        self._plot_height_name = plot_height
        self._data_name = data_name
        self._cur_dataset = None
        self._use_time_formatter = True
        try:
            self._p = figure(title=title, x_axis_label=x_axis_label,
                             y_axis_label=y_axis_label, tools=TOOLS,
                             active_scroll=ACTIVE_SCROLL_TOOLS)
            if y_range is not None:
                self._p.y_range = Range1d(y_range.start, y_range.end)
            if x_range is not None:
                # we need a copy, otherwise x-axis zooming will be synchronized
                # between all plots
                self._p.x_range = Range1d(x_range.start, x_range.end)

            if changed_params is not None:
                self._param_change_label = \
                    plot_parameter_changes(self._p, self.plot_height,
                                           changed_params)

            self._cur_dataset = [elem for elem in data
                                 if elem.name == data_name and elem.multi_id == topic_instance][0]

            if y_start is not None:
                # make sure y axis starts at y_start. We do it by adding an invisible circle
                self._p.circle(x=int(self._cur_dataset.data['timestamp'][0]),
                               y=y_start, size=0, alpha=0)

        except (KeyError, IndexError, ValueError) as error:
            print(type(error), "("+self._data_name+"):", error)
            self._had_error = True

    @property
    def title(self):
        """ return the bokeh title """
        if self._p is not None:
            title_text = self._p.title.text # pylint: disable=no-member
        else:
            title_text = ""
        return title_text

    @property
    def bokeh_plot(self):
        """ return the bokeh plot """
        return self._p

    @property
    def param_change_label(self):
        """ returns bokeh LabelSet or None """
        return self._param_change_label

    @property
    def has_param_change_labels(self):
        """ Does the plot have changed parameter labels? """
        return self._param_change_label is not None

    @property
    def had_error(self):
        """ Returns true if the previous plotting calls had an error (e.g. due
        to missing data in the log) """
        return self._had_error


    @property
    def dataset(self):
        """ get current dataset """
        return self._cur_dataset

    def change_dataset(self, data_name, topic_instance=0):
        """ select a new dataset. Afterwards, call add_graph etc """
        self._data_name = data_name
        if not self._had_error: self._previous_success = True
        self._had_error = False
        try:
            self._cur_dataset = [elem for elem in self._data
                                 if elem.name == data_name and elem.multi_id == topic_instance][0]
        except (KeyError, IndexError, ValueError) as error:
            print(type(error), "("+self._data_name+"):", error)
            self._had_error = True
            self._cur_dataset = None


    def add_graph(self, field_names, colors, legends, use_downsample=True,
                  mark_nan=False, use_step_lines=False):
        """ add 1 or more lines to a graph

        field_names can be a list of fields from the data set, or a list of
        functions with the data set as argument and returning a tuple of
        (field_name, data)
        :param mark_nan: if True, add an indicator to the plot when one of the graphs is NaN
        :param use_step_lines: if True, render step lines (after each point)
        instead of rendering a straight line to the next point
        """
        if self._had_error: return
        try:
            p = self._p
            data_set = {}
            data_set['timestamp'] = self._cur_dataset.data['timestamp']
            field_names_expanded = self._expand_field_names(field_names, data_set)

            if mark_nan:
                # look through the data to find NaN's and store their timestamps
                nan_timestamps = set()
                for key in field_names_expanded:
                    nan_indexes = np.argwhere(np.isnan(data_set[key]))
                    last_index = -2
                    for ind in nan_indexes:
                        if last_index + 1 != ind: # store only timestamps at the start of NaN
                            nan_timestamps.add(data_set['timestamp'][ind][0])
                        last_index = ind

                nan_color = 'black'
                for nan_timestamp in nan_timestamps:
                    nan_line = Span(location=nan_timestamp,
                                    dimension='height', line_color=nan_color,
                                    line_dash='dashed', line_width=3)
                    p.add_layout(nan_line)
                if len(nan_timestamps) > 0:
                    y_values = [30] * len(nan_timestamps)
                    # NaN label: add a space to separate it from the line
                    names = [' NaN'] * len(nan_timestamps)
                    source = ColumnDataSource(data=dict(x=np.array(list(nan_timestamps)),
                                                        names=names, y=y_values))
                    # plot as text with a fixed screen-space y offset
                    labels = LabelSet(x='x', y='y', text='names',
                                      y_units='screen', level='glyph', text_color=nan_color,
                                      source=source, render_mode='canvas')
                    p.add_layout(labels)


            if use_downsample:
                # we directly pass the data_set, downsample and then create the
                # ColumnDataSource object, which is much faster than
                # first creating ColumnDataSource, and then downsample
                downsample = DynamicDownsample(p, data_set, 'timestamp')
                data_source = downsample.data_source
            else:
                data_source = ColumnDataSource(data=data_set)

            for field_name, color, legend in zip(field_names_expanded, colors, legends):
                if use_step_lines:
                    p.step(x='timestamp', y=field_name, source=data_source,
                           legend=legend, line_width=2, line_color=color,
                           mode="after")
                else:
                    p.line(x='timestamp', y=field_name, source=data_source,
                           legend=legend, line_width=2, line_color=color)

        except (KeyError, IndexError, ValueError) as error:
            print(type(error), "("+self._data_name+"):", error)
            self._had_error = True

    def add_circle(self, field_names, colors, legends):
        """ add circles

        see add_graph for arguments description
        """
        if self._had_error: return
        try:
            p = self._p
            data_set = {}
            data_set['timestamp'] = self._cur_dataset.data['timestamp']
            field_names_expanded = self._expand_field_names(field_names, data_set)
            data_source = ColumnDataSource(data=data_set)

            for field_name, color, legend in zip(field_names_expanded, colors, legends):
                p.circle(x='timestamp', y=field_name, source=data_source,
                         legend=legend, line_width=2, size=4, line_color=color,
                         fill_color=None)

        except (KeyError, IndexError, ValueError) as error:
            print(type(error), "("+self._data_name+"):", error)
            self._had_error = True


    def _expand_field_names(self, field_names, data_set):
        """
        expand field names if they're a function
        """
        field_names_expanded = []
        for field_name in field_names:
            if hasattr(field_name, '__call__'):
                new_field_name, new_data = field_name(self._cur_dataset.data)
                data_set[new_field_name] = new_data
                field_names_expanded.append(new_field_name)
            else:
                data_set[field_name] = self._cur_dataset.data[field_name]
                field_names_expanded.append(field_name)
        return field_names_expanded


    def add_span(self, field_name, accumulator_func=np.mean,
                 line_color='black', line_alpha=0.5):
        """ Add a vertical line. Location is determined by accumulating a
        dataset """
        if self._had_error: return
        try:
            accumulated_data = accumulator_func(self._cur_dataset.data[field_name])
            data_span = Span(location=accumulated_data.item(),
                             dimension='width', line_color=line_color,
                             line_alpha=line_alpha, line_width=1)
            self._p.add_layout(data_span)

        except (KeyError, IndexError, ValueError) as error:
            print(type(error), "("+self._data_name+"):", error)
            self._had_error = True

    def set_use_time_formatter(self, use_formatter):
        """ configure whether the time formatter should be used """
        self._use_time_formatter = use_formatter

    def finalize(self):
        """ Call this after all plots are done. Returns the bokeh plot, or None
        on error """
        if self._had_error and not self._previous_success:
            return None
        self._setup_plot()
        return self._p

    @property
    def plot_height(self):
        """ get the height of the plot in screen pixels """
        return self._config['plot_height'][self._plot_height_name]

    def _setup_plot(self):
        plots_width = self._config['plot_width']
        plots_height = self.plot_height
        p = self._p

        p.plot_width = plots_width
        p.plot_height = plots_height

        # -> other attributes are set via theme.yaml

        # disable x grid lines
        p.xgrid.grid_line_color = None

        p.ygrid.grid_line_color = 'navy'
        p.ygrid.grid_line_alpha = 0.13
        p.ygrid.minor_grid_line_color = 'navy'
        p.ygrid.minor_grid_line_alpha = 0.05

        p.toolbar.logo = None # hide the bokeh logo (we give credit at the
                            # bottom of the page)

        #p.lod_threshold=None # turn off level-of-detail

        # axis labels: format time
        if self._use_time_formatter:
            p.xaxis[0].formatter = FuncTickFormatter(code='''
                    //func arguments: ticks, x_range
                    // assume us ticks
                    ms = Math.round(tick / 1000)
                    sec = Math.floor(ms / 1000)
                    minutes = Math.floor(sec / 60)
                    hours = Math.floor(minutes / 60)
                    ms = ms % 1000
                    sec = sec % 60
                    minutes = minutes % 60

                    function pad(num, size) {
                        var s = num+"";
                        while (s.length < size) s = "0" + s;
                        return s;
                    }

                    if (hours > 0) {
                        var ret_val = hours + ":" + pad(minutes, 2) + ":" + pad(sec,2)
                    } else {
                        var ret_val = minutes + ":" + pad(sec,2);
                    }
                    if (x_range.end - x_range.start < 4e6) {
                        ret_val = ret_val + "." + pad(ms, 3);
                    }
                    return ret_val;
                ''', args={'x_range' : p.x_range})

        # make it possible to hide graphs by clicking on the label
        p.legend.click_policy = "hide"

class DataPlotSpec(DataPlot):
    """
    A spectrogram plot.
    This does not downsample dynamically.

    A spectrogram plot is only added to the plotting page if the sampling frequency of the dataset is higher than 100Hz.
    """

    def __init__(self, data, config, data_name, x_axis_label=None,
                 y_axis_label=None, title=None, plot_height='small',
                 x_range=None, y_range=None, topic_instance=0):

        super(DataPlotSpec, self).__init__(data, config, data_name, x_axis_label=x_axis_label,
                                           y_axis_label=y_axis_label, title=title, plot_height=plot_height,
                                           x_range=x_range, y_range=y_range, topic_instance=topic_instance)

    def add_graph(self, field_names, legends, window='hann', window_length=256, noverlap=128):
        """ add a spectrogram plot to the graph

        field_names: can be a list of fields from the data set, or a list of
        functions with the data set as argument and returning a tuple of
        (field_name, data)
        legends: description for the field_names that will appear in the title of the plot
        window: the type of window to use for the frequency analysis. check scipy documentation for available window types.
        window_length: length of the analysis window in samples.
        noverlap: number of overlapping samples between windows.
        """

        if self._had_error: return
        try:
            data_set = {}
            data_set['timestamp'] = self._cur_dataset.data['timestamp']

            # calculate the sampling frequency
            # (Note: logging dropouts are not taken into account here)
            delta_t = ((data_set['timestamp'][-1] - data_set['timestamp'][0]) * 1.0e-6) / len(data_set['timestamp'])
            if delta_t < 0.000001: # avoid division by zero
                self._had_error = True
                return

            sampling_frequency = int(1.0 / delta_t)

            if sampling_frequency < 100: # require min sampling freq
                self._had_error = True
                return

            field_names_expanded = self._expand_field_names(field_names, data_set)

            # calculate the spectrogram
            psd = dict()
            for key in field_names_expanded:
                frequency, time, psd[key] = scipy.signal.spectrogram(
                    data_set[key], fs=sampling_frequency, window=window,
                    nperseg=window_length, noverlap=noverlap, scaling='density')

            # sum all psd's
            key_it = iter(psd)
            sum_psd = psd[next(key_it)]
            for key in key_it:
                sum_psd += psd[key]

            # offset = int(((1024/2.0)/250.0)*1e6)
            # scale time to microseconds and add start time as offset
            time = time * 1.0e6 + self._cur_dataset.data['timestamp'][0]

            color_mapper = LinearColorMapper(palette=viridis(256), low=-80, high=0)

            image = [10 * np.log10(sum_psd)]
            title = self.title
            for legend in legends:
                title += " " + legend
            title += " [dB]"

            # assume maximal data points per pixel at full resolution
            max_num_data_points = 2.0*self._config['plot_width']
            if len(time) > max_num_data_points:
                step_size = int(len(time) / max_num_data_points)
                time = time[::step_size]
                image[0] = image[0][:, ::step_size]

            self._p.y_range = Range1d(frequency[0], frequency[-1])
            self._p.toolbar_location = 'above'
            self._p.image(image=image, x=time[0], y=frequency[0], dw=(time[-1]-time[0]),
                          dh=(frequency[-1]-frequency[0]), color_mapper=color_mapper)
            color_bar = ColorBar(color_mapper=color_mapper,
                                 major_label_text_font_size="5pt",
                                 ticker=BasicTicker(desired_num_ticks=5),
                                 formatter=PrintfTickFormatter(format="%f"),
                                 title='[dB]',
                                 label_standoff=6, border_line_color=None, location=(0, 0))
            self._p.add_layout(color_bar, 'right')

            # add plot zoom tool that only zooms in time axis
            wheel_zoom = WheelZoomTool()
            self._p.toolbar.tools = [PanTool(), wheel_zoom, BoxZoomTool(dimensions="width"), ResetTool(), SaveTool()]   # updated_tools
            self._p.toolbar.active_scroll = wheel_zoom

        except (KeyError, IndexError, ValueError, ZeroDivisionError) as error:
            print(type(error), "(" + self._data_name + "):", error)
            self._had_error = True

class ULogException(Exception):
    """
    Exception to indicate an ULog parsing error. It is most likely a corrupt log
    file, but could also be a bug in the parser.
    """
    pass

def main():
    re_pwm = re.compile(r'.*pwm_([0-9]+).*') #For determining the pwm value from the file path

    parsed = parse_args()
    output_file(parsed.output)
    re_comp = re.compile(parsed.regexp)
    pwm_map   = eval(parsed.map)
    rpm_classes = range(0, len(pwm_map.keys()))
    plots_grid = [None] * len(rpm_classes) #Rows are the rpm class, columns are motor index (motor1 == column 0, motor 1 == column 1)
    for r in range(len(plots_grid)):
        plots_grid[r] = [None] * 2
    ulogfiles = list(filter(lambda s: re_comp.match(s), map(lambda s: str(s), Path('./').glob('**/*.ulg'))))
    msg_filter = ['sensor_combined']
    m1_pwm_map = list(pwm_map.keys())
    m2_pwm_map = [pwm_map[k] for k in m1_pwm_map]
    for ulogfile in ulogfiles:
        pwm_match = re_pwm.match(ulogfile)
        pwm = int(pwm_match.group(1))
        motor_name = parsed.motor1 if re.search(parsed.motor1, ulogfile) else parsed.motor2
        if motor_name == parsed.motor1:
            motor_class = 0
            rpm_class = m1_pwm_map.index(pwm)
        else:
            motor_class = 1
            rpm_class = m2_pwm_map.index(pwm)

        #Parse ulog file
        try:
            ulog = ULog(ulogfile, msg_filter, disable_str_exceptions=False)
        except FileNotFoundError:
            print("Error: file %s not found" % file_name)
            raise
        # catch all other exceptions and turn them into an ULogException
        except Exception as error:
            traceback.print_exception(*sys.exc_info())
            raise ULogException()

        data = ulog.data_list
        x_range_offset = (ulog.last_timestamp - ulog.start_timestamp) * 0.05
        x_range = Range1d(ulog.start_timestamp - x_range_offset, ulog.last_timestamp + x_range_offset)

        # Acceleration Spectrogram
        data_plot = DataPlotSpec(data, plot_config, 'sensor_combined',
                                y_axis_label='[Hz]', title='{} (PWM {}) Acceleration Power Spectral Density'.format(motor_name, pwm),
                                plot_height='small', x_range=x_range)
        data_plot.add_graph(['accelerometer_m_s2[0]', 'accelerometer_m_s2[1]', 'accelerometer_m_s2[2]'],
                            ['X', 'Y', 'Z'])
        if data_plot.finalize() is not None: 
            print("{}, {}".format(rpm_class,motor_class))
            plots_grid[rpm_class][motor_class] = data_plot._p

    #Output plots as a gridplot
    g = gridplot(plots_grid)
    show(g)

    return

if __name__ == "__main__":
    main()
