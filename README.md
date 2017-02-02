# KinectCalibrationLocalToWorld
* 算出したいDINDのuvPoints.txtとlocalPoints.txtとworldPoints.txtを指定し，アウトプットファイル名を適宜書き換えて実行する．
* 出力されたパラメータは https://github.com/morichang/StudySomething の中にあるTransform3dPointによって点群を世界座標へ変換することができる．
* uvPointsとworldPointsに誤差がある場合を考慮するとcv::solvePnP()じゃなくてcv::solvePnPRansac()を使いたいところ．(現在エラーで利用不可)
