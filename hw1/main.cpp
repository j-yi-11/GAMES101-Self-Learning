#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#define MY_PI (std::acos(-1))

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 
                 0, 1, 0, -eye_pos[1], 
                 0, 0, 1, -eye_pos[2],
                 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    // Edit begin

    double theta = rotation_angle/180.0 * MY_PI;

    Eigen::Matrix4f mr;
    mr << std::cos(theta),  -std::sin(theta),   0,  0,
          std::sin(theta),  std::cos(theta),    0,  0,
          0,    0,  1,  0,
          0,    0,  0,  1;
    model = mr * model;
    return model;
    // Edit end
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    // Edit begin
    float projection_angle_rad = eye_fov * 180.0 / MY_PI;

    float n = zNear;
    float f = zFar;
    float t = n * std::tan(projection_angle_rad / 2.0);
    float r = t * aspect_ratio;
    float l = -r;
    float b = -t;
    // translate matrix
    Eigen::Matrix4f Mortho_translate(4,4);
    Mortho_translate << 1, 0, 0, -(r + l) / 2,
        0, 1, 0, -(t + b) / 2,
        0, 0, 1, -(n + f) / 2,
        0, 0, 0, 1;
    // scale matrix
    Eigen::Matrix4f Mortho_scale(4,4);
    Mortho_scale << 2.0 / (r - l), 0, 0, 0,
        0, 2.0 / (t - b), 0, 0,
        0, 0, 2.0 / (f - n), 0,
        0, 0, 0, 1;
    //
    Eigen::Matrix4f Mortho_to_pers(4, 4);
    Mortho_to_pers << n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -n * f,
        0, 0, 1, 0;

    Eigen::Matrix4f Mt(4, 4);
    Mt << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1;
    Mortho_to_pers = Mortho_to_pers * Mt;
    projection = Mortho_scale * Mortho_translate * Mortho_to_pers * projection;
    return projection;

    // Edit end
}


// Edit begin
Eigen::Matrix4f get_rotation(Vector3f axis, float angel)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    axis /= axis.norm();

    double angle_x = std::acos(axis[0]);
    double angle_y = std::acos(axis[1]);
    double angle_z = std::acos(axis[2]);
    Eigen::Matrix4f m1, m2, m3 = Eigen::Matrix4f::Identity();
    m1 << 1, 0, 0, 0, 0, cos(angle_x), -sin(angle_x), 0, 0, sin(angle_x), cos(angle_x), 0, 0, 0, 0, 1;
    m2 << cos(angle_y), 0, sin(angle_y), 0, 0, 1, 0, 0, -sin(angle_y), 0, cos(angle_y), 0, 0, 0, 0, 1;
    m3 << cos(angle_z), -sin(angle_z), 0, 0, sin(angle_z), cos(angle_z), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    rotation = m3 * m2 * m1 * Eigen::Matrix4f::Identity();
    return rotation;


    ////Rodrigues’ Rotation Formula
    //float rad = angel / 180.0 * MY_PI;
    //Eigen::Matrix3f n(3, 3);
    //n << 0, -axis[2], axis[1],
    //    axis[2], 0, -axis[0],
    //    -axis[1], axis[0], 0;

    //Eigen::Matrix3f component1 = Eigen::Matrix3f::Identity() * cos(rad);
    //Eigen::Matrix3f component2 = axis * axis.transpose() * (1 - cos(rad));
    //Eigen::Matrix3f component3 = n * sin(rad);

    //Eigen::Matrix3f m_rotate = component1 + component2 + component3;

    //Eigen::Matrix4f m4_rotate = Eigen::Matrix4f::Identity();

    ////在0,0位置取3x3的矩阵
    //m4_rotate.block(0, 0, 3, 3) = m_rotate;

    //model = m4_rotate * model;

    //return model;
}
// Edit end

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    // Edit begin
    Eigen::Vector3f rotate_axis = {1,1,0};
    // Edit end

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // Edit begin
        //围绕z轴旋转
        //r.set_model(get_model_matrix(angle));
        //围绕任意轴旋转
        r.set_model(get_rotation(rotate_axis,angle));
        // Edit end

        r.set_view(get_view_matrix(eye_pos));
        //注意这里写入的zNear和zFar是正数，代表着距离，但课程上推导的透视矩阵是坐标，且假定是朝向z负半轴的，所以透视矩阵是需要取反的
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // Edit begin
        //围绕z轴旋转
        r.set_model(get_model_matrix(angle));
        //围绕任意轴旋转
        //r.set_model(get_rotation(rotate_axis,angle));
        // Edit end

        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        std::cout << "Image data type: " << image.type() << std::endl;
        std::cout << "Image data size: " << image.size() << std::endl;
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
            std::cout << "key = " << key << " angle = " << angle << std::endl;
        }
        else if (key == 'd') {
            angle -= 10;
            std::cout << "key = " << key << " angle = " << angle << std::endl;
        }
    }

    return 0;
}
