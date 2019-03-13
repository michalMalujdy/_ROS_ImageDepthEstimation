import numpy as np
import tensorflow as tf
import os

class ImageDepthNeuralNetwork():
    def __init__(self, data_dir = './src/data'):
        self.depthLim = 65535

        self.data_dir = data_dir
        self.model_meta_graph_filename = 'model-inference-513x257-0.meta'
        
        self.calibPath = '{}/{}'.format(self.data_dir, '000000.txt')
        self.calibData = self.load_cam_to_cam(self.calibPath)

        self.imWidth = self.calibData['imWidth']
        self.fx = self.calibData['K_cam2'][0, 0]
        self.baseline = self.calibData['baseline']

        self.initialize_tf_network()

    
    def initialize_tf_network(self):

        # path to .meta
        self.loader = tf.train.import_meta_graph('{}/{}'.format(self.data_dir, self.model_meta_graph_filename))

        self.graph = tf.get_default_graph()

        # numpy arrays as inputs
        self.input_img_1 = self.graph.get_tensor_by_name('Dataloader/read_image/read_png_image/DecodePng:0')
        self.input_img_2 = self.graph.get_tensor_by_name('Dataloader/read_image_1/read_png_image/DecodePng:0')
        self.disp_left = self.graph.get_tensor_by_name("disparities/ExpandDims:0")

        self.config = tf.ConfigProto(
            allow_soft_placement = True, 
            inter_op_parallelism_threads = 2, 
            intra_op_parallelism_threads = 1)

        self.tf_session = tf.Session(config = self.config)
        
        # restore model parameters
        self.loader.restore(self.tf_session, '{}/{}'.format(self.data_dir, 'model-inference-513x257-0'))
        self.merged = tf.summary.merge_all()


    def read_calib_file(self, filepath):
        """Read in a calibration file and parse into a dictionary."""
        data = {}

        with open(filepath, 'r') as f:
            for line in f.readlines():
                key, value = line.split(':', 1)
                # The only non-float values in these files are dates, which
                # we don't care about anyway
                try:
                    data[key] = np.array([float(x) for x in value.split()])
                except ValueError:
                    pass

        return data


    def load_cam_to_cam(self, filepath):
        data = {}

        filedata = self.read_calib_file(filepath)

        P_rect_20 = np.reshape(filedata['P_rect_02'], (3, 4))
        P_rect_30 = np.reshape(filedata['P_rect_03'], (3, 4))

        R_rect_20 = np.eye(4)
        R_rect_20[0:3, 0:3] = np.reshape(filedata['R_rect_02'], (3, 3))
        R_rect_30 = np.eye(4)
        R_rect_30[0:3, 0:3] = np.reshape(filedata['R_rect_03'], (3, 3))

        T2 = np.eye(4)
        T2[0, 3] = P_rect_20[0, 3] / P_rect_20[0, 0]
        T3 = np.eye(4)
        T3[0, 3] = P_rect_30[0, 3] / P_rect_30[0, 0]

        p_cam = np.array([0, 0, 0, 1])
        p_2 = np.linalg.inv(T2).dot(p_cam)
        p_3 = np.linalg.inv(T3).dot(p_cam)

        data['K_cam2'] = P_rect_20[0:3, 0:3]
        data['K_cam3'] = P_rect_30[0:3, 0:3]

        data['baseline'] = np.linalg.norm(p_3 - p_2)

        data['imWidth'] = filedata['S_rect_02'][0]
        data['imHeight'] = filedata['S_rect_02'][1]

        return data


    def estimate_depth(self, left_img, right_img):
        
        # run
        run_options = tf.RunOptions(trace_level = tf.RunOptions.FULL_TRACE)
        run_metadata = tf.RunMetadata()
        summary, disp = self.tf_session.run([self.merged, self.disp_left],
                                feed_dict = {
                                    self.input_img_1: left_img,
                                    self.input_img_2: right_img},
                                options = run_options,
                                run_metadata = run_metadata)

        # select a slice from first dimension
        disp = disp[0]

        # depth in [mm]
        depth = np.uint16(1000 * self.baseline * self.fx / (disp * self.imWidth))
        dispLim = 1000 * self.baseline * self.fx / (self.depthLim * self.imWidth)
        depth[disp < dispLim] = 0

        return depth
