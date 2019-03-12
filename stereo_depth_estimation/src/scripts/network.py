import numpy as np
import tensorflow as tf
import os
import cv2 as cv


def read_calib_file(filepath):
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


def load_cam_to_cam(filepath):
    data = {}

    filedata = read_calib_file(filepath)

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

def estimate_depth(left_img, right_img):
    depthLim = 65535

    # Display received images
    # cv.imshow("left", left_img)
    # cv.imshow("right", right_img)
    # cv.waitKey(-1)

    # paths
    #datasetPath = "/media/jachu/JanW/KITTI_2015_stereo/data_scene_flow/testing"
    #calibPath = "/media/jachu/JanW/KITTI_2015_stereo/data_scene_flow_calib/testing/calib_cam_to_cam/000000.txt"
    data_dir = '/media/dexter/Xubuntu/catkin_ws/src/stereo_depth_estimation/src/data'
    datasetPath = '{}/{}'.format(data_dir, 'dataset')
    calibPath = '{}/{}'.format(data_dir, '000000.txt')

    # calibration parameters
    calibData = load_cam_to_cam(calibPath)

    print('K_cam2', calibData['K_cam2'])
    print('K_cam3', calibData['K_cam3'])
    print('baseline', calibData['baseline'])
    print('imWidth', calibData['imWidth'])
    print('imHeight', calibData['imHeight'])

    imWidth = calibData['imWidth']
    fx = calibData['K_cam2'][0, 0]
    baseline = calibData['baseline']

    # path to .meta
    # loader = tf.train.import_meta_graph('data/model-inference-1025x321-0.meta')
    loader = tf.train.import_meta_graph('{}/{}'.format(data_dir, 'model-inference-513x257-0.meta'))

    # filename as input
    input_img_1 = tf.get_default_graph().get_tensor_by_name('Dataloader/read_image/read_png_image/DecodePng:0')
    input_img_2 = tf.get_default_graph().get_tensor_by_name('Dataloader/read_image_1/read_png_image/DecodePng:0')
    disp_left = tf.get_default_graph().get_tensor_by_name("disparities/ExpandDims:0")

    config = tf.ConfigProto(allow_soft_placement=True, inter_op_parallelism_threads=2, intra_op_parallelism_threads=1)
    with tf.Session(config=config) as sess:
        # restore model parameters
        loader.restore(sess, '{}/{}'.format(data_dir, 'model-inference-513x257-0'))

        # for graph inspection in tensorboard
        train_writer = tf.summary.FileWriter('summary', sess.graph)
        # train_writer.flush()

        # print out all variablesS
        # names = [n.name for n in tf.get_default_graph().as_graph_def().node]
        #for name in names:
        #    print(name)

        leftImFileList = sorted(os.listdir(datasetPath + "/image_2"))
        rightImFileList = sorted(os.listdir(datasetPath + "/image_3"))

        for i, entry in enumerate(leftImFileList):
            print('leftIm: ', leftImFileList[i])
            print('rightIm: ', rightImFileList[i])

            # run
            run_options = tf.RunOptions(trace_level=tf.RunOptions.FULL_TRACE)
            run_metadata = tf.RunMetadata()
            merged = tf.summary.merge_all()
            summary, disp = sess.run([merged, disp_left],
                                    feed_dict = {
                                        input_img_1: left_img,
                                        input_img_2: right_img},
                                    options = run_options,
                                    run_metadata = run_metadata)
            train_writer.add_run_metadata(run_metadata, 'run%d' % i, i)
            train_writer.add_summary(summary, i)
            # train_writer.flush()

            print('output', disp.shape)

            # select a slice from first dimension
            disp = disp[0]

            print('min disp = ', np.min(disp))
            print('max disp = ', np.max(disp))

            # depth in [mm]
            depth = np.uint16(1000 * baseline * fx / (disp * imWidth))
            dispLim = 1000 * baseline * fx / (depthLim * imWidth)
            print('dispLim ', dispLim)
            depth[disp < dispLim] = 0

            # save depth image
            """ cv.imwrite('out_depth/depth_' + leftImFileList[i], depth)"""

            # display image
            # cv.imshow("out", depth)
            # cv.waitKey(-1)

            return depth
