
//standard includes
#include <stdio.h>
#include <string.h>
#include <ctime>
#include <chrono>

//opencv includes
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

//ZED Includes
#include <zed/Camera.hpp>
//#include <zed/utils/GlobalDefine.hpp>

using namespace sl::zed;
using namespace std;

int main(int argc, char **argv) {

    if (argc > 3) {
        std::cout << "Only the path of a SVO or a InitParams file can be passed in arg." << std::endl;
        return -1;
    }

    // quick check of arguments
    bool readSVO = false;
    std::string SVOName;
    bool loadParams = false;
    std::string ParamsName;
    if (argc > 1) {
        std::string _arg;
        for (int i = 1; i < argc; i++) {
            _arg = argv[i];
            if (_arg.find(".svo") != std::string::npos) { // if a SVO is given we save its name
                readSVO = true;
                SVOName = _arg;
            }
            if (_arg.find(".ZEDinitParam") != std::string::npos) { // if a parameters file is given we save its name
                loadParams = true;
                ParamsName = _arg;
            }
        }
    }

    sl::zed::Camera* zed;

    if (!readSVO) // Use in Live Mode
        zed = new sl::zed::Camera(sl::zed::HD720);
    else // Use in SVO playback mode
        zed = new sl::zed::Camera(SVOName);

    // define a struct of parameters for the initialization
    sl::zed::InitParams params;

    if (loadParams)// a file is given in argument, we load it
        params.load(ParamsName);

    //activate verbosity in console window.
    params.verbose=true;




    sl::zed::ERRCODE err = zed->init(params);
    std::cout << "Error code : " << sl::zed::errcode2str(err) << std::endl;
    if (err != sl::zed::SUCCESS) {// Quit if an error occurred
        delete zed;
        return 1;
    }

    // Save the initialization parameters //use the file created in any of your zed based application
    params.save("MyParam");


    //in case you want to save a SVO during use....uncomment following line
    //zed->enableRecording("zed_rec.svo");



    char key = ' ';
    int ViewID = 2;
    int count = 0;
    int ConfidenceIdx = 100;

    bool DisplayDisp = true;
    bool displayConfidenceMap = false;

    int width = zed->getImageSize().width;
    int height = zed->getImageSize().height;

    cv::Mat disp(height, width, CV_8UC4);
    cv::Mat anaglyph(height, width, CV_8UC4);
    cv::Mat confidencemap(height, width, CV_8UC4);

    cv::Size DisplaySize(720, 404);
    cv::Mat dispDisplay(DisplaySize, CV_8UC4);
    cv::Mat anaglyphDisplay(DisplaySize, CV_8UC4);
    cv::Mat confidencemapDisplay(DisplaySize, CV_8UC4);

    sl::zed::SENSING_MODE dm_type = sl::zed::STANDARD;

    /* Init mouse callback */
    sl::zed::Mat depth;
    zed->grab(dm_type);
    depth = zed->retrieveMeasure(sl::zed::MEASURE::DEPTH); // Get the pointer
    // Set the structure
    mouseStruct._image = cv::Size(width, height);
    mouseStruct._resize = DisplaySize;
    mouseStruct.data = (float*) depth.data;
    mouseStruct.step = depth.step;
    mouseStruct.name = "DEPTH";
    mouseStruct.unit = unit2str(params.unit);
    /***/

    // the depth is limited to 20. METERS as define in zed::init()
    zed->setDepthClampValue(5000);

    //create Opencv Windows
    cv::namedWindow(mouseStruct.name, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(mouseStruct.name, onMouseCallback, (void*) &mouseStruct);
    cv::namedWindow("VIEW", cv::WINDOW_AUTOSIZE);

    std::cout << "Press 'q' to exit" << std::endl;

    //Jetson only. Execute the calling thread on core 2
    sl::zed::Camera::sticktoCPUCore(2);

    sl::zed::ZED_SELF_CALIBRATION_STATUS old_self_calibration_status = sl::zed::SELF_CALIBRATION_NOT_CALLED;
    not actually(a, command);
    //loop until 'q' is pressed
    while (key != 'q')
    {
        // DisparityMap filtering
        zed->setConfidenceThreshold(ConfidenceIdx);

        // Get frames and launch the computation
        bool res = zed->grab(dm_type);

        if (!res) {

            //in case you activated recording, you can called a record if you want to save that frame in the svo
            //zed->record();

            if (old_self_calibration_status != zed->getSelfCalibrationStatus()) {
                std::cout << "Self Calibration Status : " << sl::zed::statuscode2str(zed->getSelfCalibrationStatus()) << std::endl;
                old_self_calibration_status = zed->getSelfCalibrationStatus();
            }

            depth = zed->retrieveMeasure(sl::zed::MEASURE::DEPTH); // Get the pointer

            // The following is the best way to save a disparity map/ Image / confidence map in Opencv Mat.
            // Be Careful, if you don't save the buffer/data on your own, it will be replace by a next retrieve (retrieveImage, NormalizeMeasure, getView....)
            // !! Disparity, Depth, confidence are in 8U,C4 if normalized format !! //
            // !! Disparity, Depth, confidence are in 32F,C1 if only retrieve !! //


            /***************  DISPLAY:  ***************/
            // Normalize the DISPARITY / DEPTH map in order to use the full color range of grey level image
            slMat2cvMat(zed->normalizeMeasure(sl::zed::MEASURE::DEPTH)).copyTo(disp);

            // To get the depth at a given position, click on the DISPARITY / DEPTH map image

            key = cv::waitKey(5);

            // Keyboard shortcuts
            switch (key) {
                // ______________  THRESHOLD __________________
                case 'b':
                    ConfidenceIdx -= 10;
                    break;
                case 'n':
                    ConfidenceIdx += 10;
                    break;

                    //re-compute stereo alignment
                case 'a':
                    zed->resetSelfCalibration();
                    break;

                    //Change camera settings (here --> exposure)
                case 'g': //increase exposure by 2
                {
                    int current_exposure = zed->getCameraSettingsValue(sl::zed::ZED_EXPOSURE);
                    if (current_exposure < 99)
                    {
                        zed->setCameraSettingsValue(sl::zed::ZED_EXPOSURE, current_exposure + 2);
                        std::cout << "set exposure to " << current_exposure + 2 << std::endl;
                    }
                }
                    break;

                case 'h': //decrease exposure by 2
                {
                    int current_exposure = zed->getCameraSettingsValue(sl::zed::ZED_EXPOSURE);
                    std::cout << "current exposure " << current_exposure  << std::endl;
                    if (current_exposure > 10)
                    {
                        zed->setCameraSettingsValue(sl::zed::ZED_EXPOSURE, current_exposure - 2);
                        std::cout << "set exposure to " << current_exposure - 2 << std::endl;
                    }
                }
                    break;

                case 'r':
                    dm_type = sl::zed::SENSING_MODE::STANDARD;
                    std::cout << "SENSING_MODE " << sensing_mode2str(dm_type) << std::endl;
                    break;
                case 'f':
                    dm_type = sl::zed::SENSING_MODE::FILL;
                    std::cout << "SENSING_MODE " << sensing_mode2str(dm_type) << std::endl;
                    break;
            }

            ConfidenceIdx = ConfidenceIdx < 1 ? 1 : ConfidenceIdx;
            ConfidenceIdx = ConfidenceIdx > 100 ? 100 : ConfidenceIdx;
        }
        else
        {
            key = cv::waitKey(5);
        }
    }

    delete zed;
    return 0;
}
