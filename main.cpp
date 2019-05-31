#include <ov580.h>
#include <pangolin/video/video_input.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

void mouseCallback(int event, int x, int y, int flags, void *userdata) {

}

int main() {

    cvNamedWindow("Test", CV_WINDOW_NORMAL);
    cvSetWindowProperty("Test", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    cvSetMouseCallback("Test", mouseCallback);

    cv::waitKey();

    surreal::RegisterOv580VideoFactory();
    pangolin::VideoInput video("ov580:[SensorModel=7251,ExposureTime=1000,Gain=1,strobe=1000]");

    unsigned char* buffer = new unsigned char[640 * 480 * 2];
    std::vector<pangolin::Image<unsigned char>> images;

    while (true) {
        video.Grab(buffer,images,true,true);
        const auto& img = images[0];
        auto cv_img = cv::Mat(img.h,img.w,CV_8U, img.ptr, img.pitch);
        cv::imshow("test",cv_img);
        cv::waitKey(1);
    }

}