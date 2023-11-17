import matplotlib.pyplot as plt
import numpy as np
import datetime
import csv
from matplotlib.widgets import Slider, Button, TextBox
import tikzplotlib


class Plotter:
    #init
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.data = None
#attributes
    def generate_data(self, num_points=2000):
        t = np.linspace(0, 10, num_points)
        lambda_t = 5 * np.sin(2 * np.pi * 1 * t)
        h = 3 * np.pi * np.exp(-lambda_t)
        #dictionary
        self.data = {'t': t, 'h': h}
#plot config
    def plot_data(self):
        if self.data is None:
            self.generate_data()
        self.ax.plot(self.data['t'], self.data['h'])
        plt.show()

    def update_data(self):
        pass
#axis
    def adjust_axes(self, new_xlim=None, new_ylim=None):
        if new_xlim:
            self.ax.set_xlim(new_xlim)
        if new_ylim:
            self.ax.set_ylim(new_ylim)
        plt.draw()
        
#save to csv/i got help for this one 
    def save_data_as_csv(self, experiment_name):
        if self.data:
            current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = f"{experiment_name}_{current_time}.csv"
            with open(filename, 'w', newline='') as csvfile:
                fieldnames = ['t', 'h']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                for i in range(len(self.data['t'])):
                    writer.writerow({'t': self.data['t'][i], 'h': self.data['h'][i]})
            print(f"Data saved as {filename}")
            return filename



#advanced child
class AdvancedPlotter(Plotter):
    def __init__(self):
        #super is used for child and invokes the init from plotter
        super().__init__()
        self.experiment_name = "Default"
        self.is_running = False
        self.animation = None
        self.create_gui_elements()
        self.plot_data()

    def create_gui_elements(self):
        # sliders and colors
        ax_color = 'red'
        ax_slider_t = plt.axes([0.1, 0.05, 0.65, 0.03], facecolor=ax_color)
        ax_slider_lambda = plt.axes([0.1, 0.01, 0.65, 0.03], facecolor=ax_color)

        self.slider_t = Slider(ax_slider_t, 'Time', 0, 10, valinit=0)
        self.slider_lambda = Slider(ax_slider_lambda, 'Lambda', 0, 10, valinit=0)
        
        # buttons etc.
        #location
        ax_start = plt.axes([0.8, 0.15, 0.1, 0.04])
        ax_stop = plt.axes([0.8, 0.1, 0.1, 0.04])
        ax_reset = plt.axes([0.8, 0.05, 0.1, 0.04])
        ax_experiment = plt.axes([0.1, 0.9, 0.3, 0.04])
        ax_save = plt.axes([0.8, 0.9, 0.1, 0.04])
        ax_grid = plt.axes([0.8, 0.2, 0.1, 0.04])
        
        #buttons action
        self.button_start = Button(ax_start, 'Start')
        self.button_stop = Button(ax_stop, 'Stop')
        self.button_reset = Button(ax_reset, 'Reset')
        self.text_experiment = TextBox(ax_experiment, 'Run', initial=self.experiment_name)
        self.button_save = Button(ax_save, 'Save')
        self.button_grid = Button(ax_grid, 'Grid On')

        # callbacks on clicks
        self.slider_t.on_changed(self.update_data)
        self.slider_lambda.on_changed(self.update_data)
        self.button_start.on_clicked(self.start_animation)
        self.button_stop.on_clicked(self.stop_animation)
        self.button_reset.on_clicked(self.reset_data)
        self.text_experiment.on_submit(self.set_experiment_name)
        self.button_save.on_clicked(self.save_data)
        self.button_grid.on_clicked(self.toggle_grid)

    def update_sliders(val):
        self.update_data(val)
        self.slider_t.on_changed(update_sliders)
        self.slider_lambda.on_changed(update_sliders)
    
        
    def update_data(self, val=None):
        #this one was the most tricky to make it work and probably the most important one
        if self.is_running:
            t_val = self.slider_t.val
            lambda_val = self.slider_lambda.val
            t = np.linspace(0, 10, 2000)
            lambda_t = 5 * np.sin(2 * np.pi * lambda_val * t)
            h = 3 * np.pi * np.exp(-lambda_t)
            #dictionary again
            self.data = {'t': t, 'h': h}

            self.ax.clear()
            self.ax.plot(self.data['t'], self.data['h'])
            plt.draw()
        #initiation of animation
    def start_animation(self, event):
        self.is_running = True
        if not self.animation:
            self.animation = self.fig.canvas.new_timer(interval=100)
            self.animation.add_callback(self.update_data)
        self.animation.start()

    def stop_animation(self, event):
        self.is_running = False
        if self.animation:
            self.animation.stop()

    def reset_data(self, event):
        self.data = None
        self.slider_t.reset()
        self.slider_lambda.reset()
        #i dont like it that when i reset the plot doesnt go empty
        self.fig.clear()
        self.ax = self.fig.add_subplot(111)
        self.create_gui_elements()
        self.plot_data()

    def set_experiment_name(self, text):
        self.experiment_name = text

    def save_data(self, event):
        if self.data:
            current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = f"{self.experiment_name}_{current_time}.csv"
            with open(filename, 'w', newline='') as csvfile:
                fieldnames = ['t', 'h']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writerow({'t': self.data['t'], 'h': self.data['h']})
            print(f"Data saved as {filename}")
            return filename

    def toggle_grid(self, event):
        self.ax.grid(not self.ax.yaxis.get_visible())
        plt.draw()

#ask which one the user wants
ask=int(input("Press 1 for Basic and 2 for Advanced"))

if ask == 1: 
    basic_plotter = Plotter()
    basic_plotter.plot_data()
    
elif ask == 2:
    advanced_plotter = AdvancedPlotter()
else:
    print("Please choose between 1 and 2")
    exit()
