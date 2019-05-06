#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <boost/thread.hpp>
#include <string>
#include <iostream>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <pthread.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <math.h>

#include <std_msgs/Header.h>

#include <openpose_ros_msgs/BoundingBox.h>
#include <openpose_ros_msgs/OpenPoseHuman.h>
#include <openpose_ros_msgs/OpenPoseHumanList.h>
#include <openpose_ros_msgs/PointWithProb.h>


#define OPENPOSE_FLAGS_DISABLE_PRODUCER
#define OPENPOSE_FLAGS_DISABLE_DISPLAY
#include <openpose/flags.hpp>
// OpenPose dependencies
#include <openpose/headers.hpp>

DEFINE_bool(no_display,                 false,
    "Enable to disable the visual display.");

using namespace cv;
using namespace std;


class OpRosWrapper
{
    public:
    OpRosWrapper(op::Wrapper &opWrapper): it(nh), sizeColor(960, 540)
    {
        opWrapper_ = &opWrapper;
        string color_topic_1 = "/kinect2_1/qhd/image_color_rect";
        image_sub_1 = it.subscribe(color_topic_1.c_str(), 1, &OpRosWrapper::ImageProcessingCallback_1,this);
        string color_topic_2 = "/kinect2_2/qhd/image_color_rect";
        image_sub_2 = it.subscribe(color_topic_2.c_str(), 1, &OpRosWrapper::ImageProcessingCallback_2,this);
        openpose_human_list_pub_1 = nh.advertise<openpose_ros_msgs::OpenPoseHumanList>("/openpose_ros/human_list_1", 10);
        openpose_human_list_pub_2 = nh.advertise<openpose_ros_msgs::OpenPoseHumanList>("/openpose_ros/human_list_2", 10);

        image_1 = Mat(sizeColor, CV_8UC3);
        image_2 = Mat(sizeColor, CV_8UC3);
        spliced_image = Mat(1080, 960, CV_8UC3);

    };

    void ImageProcessingCallback_1(const sensor_msgs::ImageConstPtr& msg){
      try
      {
        Mat color_mat = cv_bridge::toCvShare(msg, "bgr8")->image;
        image_1 = color_mat.clone();
        /**
        Mat displyImg = color_mat.clone();
        cv::resize(displyImg, displyImg, Size(960, 540));

        auto datumProcessed = opWrapper_->emplaceAndPop(displyImg);
        if (datumProcessed != nullptr)
        {
            printKeypoints(datumProcessed);
            if (!FLAGS_no_display)
                display(datumProcessed);
            publish(datumProcessed);
        }
        else
            op::log("Image could not be processed.", op::Priority::High);
        **/
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      }
    };

    void ImageProcessingCallback_2(const sensor_msgs::ImageConstPtr& msg){
      try
      {
        Mat color_mat = cv_bridge::toCvShare(msg, "bgr8")->image;
        image_2 = color_mat.clone();
        /**
        Mat displyImg = color_mat.clone();
        cv::resize(displyImg, displyImg, Size(960, 540));

        auto datumProcessed = opWrapper_->emplaceAndPop(displyImg);
        if (datumProcessed != nullptr)
        {
            printKeypoints(datumProcessed);
            if (!FLAGS_no_display)
                display(datumProcessed);
            publish(datumProcessed);
        }
        else
            op::log("Image could not be processed.", op::Priority::High);
        **/
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      }
    };

    void run()
    {
        ros::Rate rate(60);
        while(ros::ok())
        {
            imageProcess();
            ros::spinOnce();
            rate.sleep();
        }
    }



    private:
        op::Wrapper* opWrapper_;
        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        image_transport::Subscriber image_sub_1;
        image_transport::Subscriber image_sub_2;

        ros::Publisher openpose_human_list_pub_1;
        ros::Publisher openpose_human_list_pub_2;
        std_msgs::Header image_header_;

        cv::Mat image_1;
        cv::Mat image_2;

        cv::Size sizeColor;

        cv::Mat spliced_image;

        void imageProcess()
        {
            if(!image_1.empty() && !image_2.empty())
            {
              Rect upper_piece = Rect(0, 0, 960, 540);
              Rect lower_piece = Rect(0, 540, 960, 540);
              image_1.copyTo(spliced_image(upper_piece));
              image_2.copyTo(spliced_image(lower_piece));

              auto datumProcessed = opWrapper_->emplaceAndPop(spliced_image);
              if (datumProcessed != nullptr)
              {
                  //printKeypoints(datumProcessed);
                  if (!FLAGS_no_display)
                      display(datumProcessed);
                  publish(datumProcessed);
              }
              else
              {
                  op::log("Image could not be processed.", op::Priority::High);
              }
            }
        }

        void display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
        {
            try
            {
                // User's displaying/saving/other processing here
                    // datum.cvOutputData: rendered frame with pose or heatmaps
                    // datum.poseKeypoints: Array<float> with the estimated pose
                if (datumsPtr != nullptr && !datumsPtr->empty())
                {
                    // Display image
                    cv::imshow(OPEN_POSE_NAME_AND_VERSION + " - Tutorial C++ API", datumsPtr->at(0)->cvOutputData);
                    cv::waitKey(20);
                }
                else
                    op::log("Nullptr or empty datumsPtr found.", op::Priority::High);
            }
            catch (const std::exception& e)
            {
                op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            }
        };

        void printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
        {
            try
            {
                // Example: How to use the pose keypoints
                if (datumsPtr != nullptr && !datumsPtr->empty())
                {
                    // Alternative 1
                    op::log("Body keypoints: " + datumsPtr->at(0)->poseKeypoints.toString(), op::Priority::High);

                    // // Alternative 2
                    // op::log(datumsPtr->at(0).poseKeypoints, op::Priority::High);

                    // // Alternative 3
                    // std::cout << datumsPtr->at(0).poseKeypoints << std::endl;

                    // // Alternative 4 - Accesing each element of the keypoints
                    // op::log("\nKeypoints:", op::Priority::High);
                    // const auto& poseKeypoints = datumsPtr->at(0).poseKeypoints;
                    // op::log("Person pose keypoints:", op::Priority::High);
                    // for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
                    // {
                    //     op::log("Person " + std::to_string(person) + " (x, y, score):", op::Priority::High);
                    //     for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
                    //     {
                    //         std::string valueToPrint;
                    //         for (auto xyscore = 0 ; xyscore < poseKeypoints.getSize(2) ; xyscore++)
                    //             valueToPrint += std::to_string(   poseKeypoints[{person, bodyPart, xyscore}]   ) + " ";
                    //         op::log(valueToPrint, op::Priority::High);
                    //     }
                    // }
                    // op::log(" ", op::Priority::High);
                }
                else
                    op::log("Nullptr or empty datumsPtr found.", op::Priority::High);
            }
            catch (const std::exception& e)
            {
                op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            }
        };

        void publish(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
        {
            if (datumsPtr != nullptr && !datumsPtr->empty())
            {
                const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
                /*
                const auto& faceKeypoints = datumsPtr->at(0).faceKeypoints;
                const auto& leftHandKeypoints = datumsPtr->at(0).handKeypoints[0];
                const auto& rightHandKeypoints = datumsPtr->at(0).handKeypoints[1];
                std::vector<op::Rectangle<float>>& face_rectangles = datumsPtr->at(0).faceRectangles;
                */
                openpose_ros_msgs::OpenPoseHumanList human_list_msg_1;
                human_list_msg_1.header.stamp = ros::Time::now();
                human_list_msg_1.image_header = image_header_;

                openpose_ros_msgs::OpenPoseHumanList human_list_msg_2;
                human_list_msg_2.header.stamp = ros::Time::now();
                human_list_msg_2.image_header = image_header_;
                //human_list_msg_2.num_humans = poseKeypoints.getSize(0);

                //std::vector<openpose_ros_msgs::OpenPoseHuman> human_list(poseKeypoints.getSize(0));

                //seprate keypoints for image 1,2
                vector<int> indexs_1;
                vector<int> indexs_2;
                for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
                {
                    int count = 0;
                    double y_sum = 0.0;
                    for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
                    {
                      if(poseKeypoints[{person, bodyPart, 2}] > 0.0)
                      {
                        y_sum += poseKeypoints[{person, bodyPart, 1}];
                        count ++;
                      }
                    }
                    double y_mean = y_sum/count;
                    if(y_mean < 540.0){
                        indexs_1.push_back(person);
                    }
                    else if(y_mean > 540.0 && y_mean < 1080.0){
                        indexs_2.push_back(person);
                    }
                }
                human_list_msg_1.num_humans = indexs_1.size();
                human_list_msg_2.num_humans = indexs_2.size();
                int person_num = indexs_1.size();
                ROS_INFO("Person in Image 1: %d\n", person_num);
                person_num = indexs_2.size();
                ROS_INFO("Person in Image 2: %d\n\n", person_num);


                //Image 1 Keypoints
                std::vector<openpose_ros_msgs::OpenPoseHuman> human_list_1(indexs_1.size());
                for (auto index = 0 ; index < indexs_1.size() ; index++)
                {
                    int person = indexs_1[index];
                    openpose_ros_msgs::OpenPoseHuman human;

                    int num_body_key_points_with_non_zero_prob = 0;
                    cout << "keypoints num: " << poseKeypoints.getSize(1) << endl;
                    for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
                    {
                        openpose_ros_msgs::PointWithProb body_point_with_prob;
                        body_point_with_prob.x = poseKeypoints[{person, bodyPart, 0}];
                        body_point_with_prob.y = poseKeypoints[{person, bodyPart, 1}];
                        body_point_with_prob.prob = poseKeypoints[{person, bodyPart, 2}];
                        if(body_point_with_prob.prob > 0)
                        {
                            num_body_key_points_with_non_zero_prob++;
                        }
                        human.body_key_points_with_prob.at(bodyPart) = body_point_with_prob;
                    }
                    human.num_body_key_points_with_non_zero_prob = num_body_key_points_with_non_zero_prob;
                    human_list_1.at(index) = human;
                }
                //ROS_INFO("log1_1");


                human_list_msg_1.human_list = human_list_1;
                openpose_human_list_pub_1.publish(human_list_msg_1);

                //Image 2 Keypoints
                std::vector<openpose_ros_msgs::OpenPoseHuman> human_list_2(indexs_2.size());
                for (auto index = 0 ; index < indexs_2.size() ; index++)
                {
                    int person = indexs_2[index];
                    openpose_ros_msgs::OpenPoseHuman human;

                    int num_body_key_points_with_non_zero_prob = 0;
                    cout << "keypoints num: " << poseKeypoints.getSize(1) << endl;
                    for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
                    {
                        openpose_ros_msgs::PointWithProb body_point_with_prob;
                        body_point_with_prob.x = poseKeypoints[{person, bodyPart, 0}];
                        body_point_with_prob.y = poseKeypoints[{person, bodyPart, 1}] - 540;
                        body_point_with_prob.prob = poseKeypoints[{person, bodyPart, 2}];
                        if(body_point_with_prob.prob > 0)
                        {
                            num_body_key_points_with_non_zero_prob++;
                        }
                        //ROS_INFO("log0");
                        human.body_key_points_with_prob.at(bodyPart) = body_point_with_prob;
                    }
                    //ROS_INFO("log1");
                    human.num_body_key_points_with_non_zero_prob = num_body_key_points_with_non_zero_prob;
                    human_list_2.at(index) = human;
                }
                //ROS_INFO("log2");
                human_list_msg_2.human_list = human_list_2;
                openpose_human_list_pub_2.publish(human_list_msg_2);

            }
            else
                op::log("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
        }

};

void configureWrapper(op::Wrapper& opWrapper)
{
    try
    {
        // Configuring OpenPose

        // logging_level
        op::check(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.",
                  __LINE__, __FUNCTION__, __FILE__);
        op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
        op::Profiler::setDefaultX(FLAGS_profile_speed);

        // Applying user defined configuration - GFlags to program variables
        // outputSize
        const auto outputSize = op::flagsToPoint(FLAGS_output_resolution, "-1x-1");
        // netInputSize
        const auto netInputSize = op::flagsToPoint(FLAGS_net_resolution, "-1x368");
        // faceNetInputSize
        const auto faceNetInputSize = op::flagsToPoint(FLAGS_face_net_resolution, "368x368 (multiples of 16)");
        // handNetInputSize
        const auto handNetInputSize = op::flagsToPoint(FLAGS_hand_net_resolution, "368x368 (multiples of 16)");
        // poseMode
        const auto poseMode = op::flagsToPoseMode(FLAGS_body);
        // poseModel
        const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);
        // JSON saving
        if (!FLAGS_write_keypoint.empty())
            op::log("Flag `write_keypoint` is deprecated and will eventually be removed."
                    " Please, use `write_json` instead.", op::Priority::Max);
        // keypointScaleMode
        const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);
        // heatmaps to add
        const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
                                                      FLAGS_heatmaps_add_PAFs);
        const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
        // >1 camera view?
        const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1);
        // Face and hand detectors
        const auto faceDetector = op::flagsToDetector(FLAGS_face_detector);
        const auto handDetector = op::flagsToDetector(FLAGS_hand_detector);
        // Enabling Google Logging
        const bool enableGoogleLogging = true;

        // Pose configuration (use WrapperStructPose{} for default and recommended configuration)
        const op::WrapperStructPose wrapperStructPose{
            poseMode, netInputSize, outputSize, keypointScaleMode, FLAGS_num_gpu, FLAGS_num_gpu_start,
            FLAGS_scale_number, (float)FLAGS_scale_gap, op::flagsToRenderMode(FLAGS_render_pose, multipleView),
            poseModel, !FLAGS_disable_blending, (float)FLAGS_alpha_pose, (float)FLAGS_alpha_heatmap,
            FLAGS_part_to_show, FLAGS_model_folder, heatMapTypes, heatMapScaleMode, FLAGS_part_candidates,
            (float)FLAGS_render_threshold, FLAGS_number_people_max, FLAGS_maximize_positives, FLAGS_fps_max,
            FLAGS_prototxt_path, FLAGS_caffemodel_path, (float)FLAGS_upsampling_ratio, enableGoogleLogging};
        opWrapper.configure(wrapperStructPose);
        // Face configuration (use op::WrapperStructFace{} to disable it)
        const op::WrapperStructFace wrapperStructFace{
            FLAGS_face, faceDetector, faceNetInputSize,
            op::flagsToRenderMode(FLAGS_face_render, multipleView, FLAGS_render_pose),
            (float)FLAGS_face_alpha_pose, (float)FLAGS_face_alpha_heatmap, (float)FLAGS_face_render_threshold};
        opWrapper.configure(wrapperStructFace);
        // Hand configuration (use op::WrapperStructHand{} to disable it)
        const op::WrapperStructHand wrapperStructHand{
            FLAGS_hand, handDetector, handNetInputSize, FLAGS_hand_scale_number, (float)FLAGS_hand_scale_range,
            op::flagsToRenderMode(FLAGS_hand_render, multipleView, FLAGS_render_pose), (float)FLAGS_hand_alpha_pose,
            (float)FLAGS_hand_alpha_heatmap, (float)FLAGS_hand_render_threshold};
        opWrapper.configure(wrapperStructHand);
        // Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
        const op::WrapperStructExtra wrapperStructExtra{
            FLAGS_3d, FLAGS_3d_min_views, FLAGS_identification, FLAGS_tracking, FLAGS_ik_threads};
        opWrapper.configure(wrapperStructExtra);
        // Output (comment or use default argument to disable any output)
        const op::WrapperStructOutput wrapperStructOutput{
            FLAGS_cli_verbose, FLAGS_write_keypoint, op::stringToDataFormat(FLAGS_write_keypoint_format),
            FLAGS_write_json, FLAGS_write_coco_json, FLAGS_write_coco_json_variants, FLAGS_write_coco_json_variant,
            FLAGS_write_images, FLAGS_write_images_format, FLAGS_write_video, FLAGS_write_video_fps,
            FLAGS_write_video_with_audio, FLAGS_write_heatmaps, FLAGS_write_heatmaps_format, FLAGS_write_video_3d,
            FLAGS_write_video_adam, FLAGS_write_bvh, FLAGS_udp_host, FLAGS_udp_port};
        opWrapper.configure(wrapperStructOutput);
        // No GUI. Equivalent to: opWrapper.configure(op::WrapperStructGui{});
        // Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
        if (FLAGS_disable_multi_thread)
            opWrapper.disableMultiThreading();
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}

int main(int argc, char *argv[])
{
    // Parsing command line flags
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc, argv, "image_display");

    op::log("Starting OpenPose demo...", op::Priority::High);
    const auto opTimer = op::getTimerInit();

    // Configuring OpenPose
    op::log("Configuring OpenPose...", op::Priority::High);
    op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
    // Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
    configureWrapper(opWrapper);
    if (FLAGS_disable_multi_thread)
        opWrapper.disableMultiThreading();

    // Starting OpenPose
    op::log("Starting thread(s)...", op::Priority::High);
    opWrapper.start();

    OpRosWrapper opRosWrapper(opWrapper);

    opRosWrapper.run();

    //ros::spin();

    opWrapper.stop();
    // Running tutorialApiCpp
    return 0;
}
