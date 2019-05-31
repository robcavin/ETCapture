#include <ov580.h>
#include <pangolin/video/video_input.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <linux/usb/video.h>
#include <linux/uvcvideo.h>


void mouseCallback(int event, int x, int y, int flags, void *userdata) {
}

int main() {

    cvNamedWindow("Test", CV_WINDOW_NORMAL);
    //cvSetWindowProperty("Test", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    cvSetMouseCallback("Test", mouseCallback);

    //cv::waitKey();

    surreal::RegisterOv580VideoFactory();
    pangolin::VideoInput video;
    bool haveVideo = false;
    try {
        video.Open("ov580:[SensorModel=7251,ExposureTime=1000,Gain=1,strobe=1000]");
        haveVideo = true;
    } catch (std::exception e) {
        printf("Failed to open camera\n");
    }

    unsigned char* buffer = new unsigned char[640 * 480 * 2];
    std::vector<pangolin::Image<unsigned char>> images;

    bool run = true;

    while (run) {
        cv::Mat cv_img;
        if (haveVideo) {
            video.Grab(buffer,images,true,true);
            const auto& img = images[0];
            cv_img = cv::Mat(img.h,img.w,CV_8U, img.ptr, img.pitch);
        } else {
            cv_img = cv::Mat(480,1280,CV_8U,cv::Scalar(127));
        }

        cv::imshow("test",cv_img);
        cv::waitKey(1);
    }

}