#include <ov580.h>
#include <pangolin/video/video_input.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>

bool record = false;
int mouse_x, mouse_y;

void mouseCallback(int event, int x, int y, int flags, void *userdata) {
    if( event == cv::EVENT_LBUTTONDOWN ) record = true;
    if( event == cv::EVENT_LBUTTONUP ) record = false;
    mouse_x = x;
    mouse_y = y;
}

int main() {

    cvNamedWindow("ETCapture", CV_WINDOW_NORMAL);
    cvSetWindowProperty("ETCapture", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

    auto window_size = cv::Size(2560,1600);

    cvSetMouseCallback("ETCapture", mouseCallback);

    auto target = cv::imread("../target.png");
    cv::resize(target,target,cv::Size(64,64));

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

    int image_number = 1;

    while (run) {

        // Update eye image
        cv::Mat eye_image;
        if (haveVideo) {
            video.Grab(buffer,images,true,true);
            const auto& img = images[0];
            eye_image = cv::Mat(img.h,img.w * 2,CV_8U, img.ptr, img.pitch);
        } else {
            eye_image = cv::Mat(480,1280,CV_8U,cv::Scalar(256));
        }
        cv::imshow("Eye image",eye_image);

        cv::Mat background(window_size.height, window_size.width, CV_8U, cv::Scalar(127));
        cv::Rect rect(mouse_x - target.size().width/2,mouse_y - target.size().height/2, target.size().width, target.size().height);
        background(rect & cv::Rect({0,0},window_size)) = 0;
        cv::imshow("ETCapture",background);
        cv::waitKey(20);

        if (record) {
            char filename[64];
            sprintf(filename, "img%06d.jpg", image_number);
            cv::imwrite(filename, eye_image);
            sprintf(filename, "img%06d.json", image_number);
            auto file = fopen(filename, "w+");
            fprintf(file, "{\"pt\":[%d,%d]}", mouse_x, mouse_y);
            image_number++;
        }
    }

}