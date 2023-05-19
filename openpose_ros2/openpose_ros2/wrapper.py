import pyopenpose as op
import numpy as np


class OpenPoseWrapper:
    """OpenPoseのROS2 Wrapper."""

    params: dict

    def __init__(self, openpose_path: str = '~/openpose'):
        """
        コンストラクタ.

        :param openpose_path: openposeが格納されているpath
        """

        self.params = dict()
        self.params['model_folder'] = openpose_path + '/models'
        #self.params['net_resolution'] = "160x80"
        #self.params['face_net_resolution'] = "256x256"
        #self.params['hand_net_resolution'] = "128x128"
        #self.params['model_pose'] = "COCO"
        #self.params['face'] = True
        #self.params['face_detector'] = 0
        self.params['hand'] = True
        self.params['hand_detector'] = 0
        self.params['body'] = 1
        

        # Config OpenWrapper
        self.op_wrapper = op.WrapperPython()
        self.op_wrapper.configure(self.params)
        self.op_wrapper.start()

    def body_from_image(self, image: np.ndarray) -> op.Datum:
        """
        画像から骨格を推定します.

        :param image: 画像
        :return: 推定データ
        """
        # 推定
        datum = op.Datum()
        datum.cvInputData = image
        self.op_wrapper.emplaceAndPop(op.VectorDatum([datum]))

        return datum
