from metavision_sdk_core import PeriodicFrameGenerationAlgorithm
from metavision_core.event_io import EventsIterator
from metavision_core.event_io.raw_reader import initiate_device

class PropheseeReader:

    def __init__(self, input_path):

        # Initialize Prophesee device
        device = initiate_device(input_path)

        # Initialize event iterator
        self.mv_iterator = EventsIterator.from_device(device=device)
        bias = self.mv_iterator.reader.device.get_i_ll_biases()

        # Initialize frame generator
        self.height, self.width = self.mv_iterator.get_size()
        self.event_frame_gen = PeriodicFrameGenerationAlgorithm(sensor_width=self.width, sensor_height=self.height, accumulation_time_us=132000)
        self.event_frame_gen.set_output_callback(self.on_cd_frame_cb)

        # Set device biases
        # device.get_i_ll_biases().set("bias_diff",0)
        device.get_i_ll_biases().set("bias_diff_off",20)
        device.get_i_ll_biases().set("bias_diff_on",20)
        device.get_i_ll_biases().set("bias_fo",-35)
        device.get_i_ll_biases().set("bias_hpf",30)

        print("Camera Initialized, Biases set.")

    def on_cd_frame_cb(self, ts, cd_frame):
        raise NotImplementedError("This method must be implemented by the subclass")

    def run(self):
        # Process events
        for evs in self.mv_iterator:
            # Dispatch system events to the window
            # EventLoop.poll_and_dispatch()
            self.event_frame_gen.process_events(evs)