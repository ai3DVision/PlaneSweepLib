// This file is part of PlaneSweepLib (PSL)

// Copyright 2016 Christian Haene (ETH Zuerich)

// PSL is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// PSL is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with PSL.  If not, see <http://www.gnu.org/licenses/>.

#include <boost/program_options.hpp>
#include <psl_base/exception.h>
#include <fstream>
#include <Eigen/Dense>
#include <psl_base/cameraMatrix.h>
#include <psl_stereo/cudaPlaneSweep.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <vector>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

void makeOutputFolder(std::string folderName)
{
    if (!boost::filesystem::exists(folderName))
    {
        if (!boost::filesystem::create_directory(folderName))
        {
            std::stringstream errorMsg;
            errorMsg << "Could not create output directory: " << folderName;
            PSL_THROW_EXCEPTION(errorMsg.str().c_str());
        }
    }
}

bool has_suffix(const std::string &str, const std::string &suffix)
{
    return str.size() >= suffix.size() &&
               str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

void get_files_in_folder(std::string folder_name, std::vector<std::string> &files, std::string suffix)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(folder_name.c_str())) == NULL) {
        std::cout << "Error(" << errno << ") opening " << folder_name << std::endl;
    }

    while ((dirp = readdir(dp)) != NULL) {
        std::string f = std::string(dirp->d_name);
        if (has_suffix(f, suffix) != 0)
            files.push_back(f);
    }
    closedir(dp);
}

std::vector<int> get_adjacent_views(int reference_view, int n_views, int N) {
    std::vector<int> adjacent_views_idxs;
    
    // TODO; Fix this and make it less hacky!!!
    if (reference_view == 0)
    {
        for (int idx = 1; idx < 1 + n_views; idx++)
            adjacent_views_idxs.push_back(idx);
    }
    else if (reference_view == 1)
    {
        adjacent_views_idxs.push_back(0);
        for (int idx = 2; idx < 1+ n_views; idx++)
            adjacent_views_idxs.push_back(idx);
    }
    else if (reference_view == N-1)
    {
        // This practically corresponds to the last avaialable view, because we
        // start counting from 0
        for (int idx = N-2; idx >= N - 1- n_views; idx--)
            adjacent_views_idxs.push_back(idx);
    }
    else if (reference_view == N-2)
    {
        adjacent_views_idxs.push_back(N-1);
        for (int idx = N-3; idx > N - 2 -n_views; idx--)
            adjacent_views_idxs.push_back(idx);
    }
    else{
        int median = n_views/2;
        for (int idx=reference_view - median; idx < reference_view; idx++)
            adjacent_views_idxs.push_back(idx);

        for (int idx=reference_view + 1; idx <= reference_view + median; idx++)
            adjacent_views_idxs.push_back(idx);
    }
    return adjacent_views_idxs;
}

void read_camera_parameters(std::string camera_parameter_file, Eigen::Matrix3d &K, Eigen::Matrix<double, 3, 3> &R, Eigen::Matrix<double, 3, 1> &T)
{
    std::fstream myfile(camera_parameter_file.c_str(), std::ios_base::in);
    std::vector<float> input_values;

    float a;
    while (myfile >> a)
        input_values.push_back(a);

    // Update the values of the K matrix
    K(0, 0) = input_values[0];
    K(0, 1) = input_values[1];
    K(0, 2) = input_values[2];
    K(1, 0) = input_values[3];
    K(1, 1) = input_values[4];
    K(1, 2) = input_values[5];
    K(2, 0) = input_values[6];
    K(2, 1) = input_values[7];
    K(2, 2) = input_values[8];

    // Update the values of R matrix
    R(0, 0) = input_values[9];
    R(0, 1) = input_values[10];
    R(0, 2) = input_values[11];
    R(1, 0) = input_values[12];
    R(1, 1) = input_values[13];
    R(1, 2) = input_values[14];
    R(2, 0) = input_values[15];
    R(2, 1) = input_values[16];
    R(2, 2) = input_values[17];

    // Update the values of T matrix
    T(0, 0) = input_values[18];
    T(1, 0) = input_values[19];
    T(2, 0) = input_values[20];
}

int main(int argc, char* argv[])
{
    std::string dataFolder;
    std::string output_folder;
    int reference_idx;
    int n_views;

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
            ("help", "Produce help message")
            ("dataFolder", boost::program_options::value<std::string>(&dataFolder), "Path to the data")
            ("output_folder", boost::program_options::value<std::string>(&output_folder), "Path to the save tha output data")
            ("reference_idx", boost::program_options::value<int>(&reference_idx), "Reference frame to be used")
            ("n_views", boost::program_options::value<int>(&n_views), "Number of views to be used")
            ;

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).run(), vm);
    boost::program_options::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    std::string camera_parameters_folder = dataFolder + "/cams_krt/";

    std::vector<std::string> camera_files;
    get_files_in_folder(camera_parameters_folder, camera_files, ".txt");
    // Make sure that the vector containing the files is sorted
    std::sort(camera_files.begin(), camera_files.end());

    int numCameras = camera_files.size();
    std::vector<int> adjacent_views_idxs = get_adjacent_views(reference_idx, n_views, numCameras);

    std::map<int, PSL::CameraMatrix<double> > cameras;
    for (int c = 0; c < numCameras; c++)
    {
        std::string camera_file = camera_parameters_folder + camera_files[c];

        Eigen::Matrix<double, 3, 3> R;
        Eigen::Matrix<double, 3, 1> T;
        Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
        read_camera_parameters(camera_file, K, R, T);

        cameras[c].setKRT(K, R, T);
    }

    std::string images_folder = dataFolder + "/imgs/";

    std::vector<std::string> image_files;
    get_files_in_folder(images_folder, image_files, ".png");
    // Make sure that the vector containing the files is sorted
    std::sort(image_files.begin(), image_files.end());

    double avgDistance = 0;
    int numDistances = 0;

    for (unsigned int i = 0; i < image_files.size()-1; i++)
    {
        if (cameras.count(i) == 1)
        {
            for (unsigned int j = i + 1; j < image_files.size(); j++)
            {
                if (cameras.count(j) == 1)
                {
                    Eigen::Vector3d distance = cameras[i].getC() - cameras[j].getC();

                    avgDistance += distance.norm();
                    numDistances++;
                }
            }
        }
    }

    avgDistance /= numDistances;
    std::cout << "Cameras have an average distance of " << avgDistance << "." << std::endl;

    float minZ = (float) (0.5f*avgDistance);
    float maxZ = (float) (1.5f*avgDistance);
    std::cout << "  Z range :  " << minZ << "  - " << maxZ <<  std::endl;
    makeOutputFolder("pinholeTestResults");

    // First tests compute a depth map for the middle image of the first row
    {
        makeOutputFolder("pinholeTestResults/colorSAD");

        PSL::CudaPlaneSweep cPS;
        cPS.setScale(0.75); // Scale the images down to 0.25 times the original side length
        cPS.setZRange(minZ, maxZ);
        cPS.setMatchWindowSize(7,7);
        cPS.setNumPlanes(256);
        cPS.setOcclusionMode(PSL::PLANE_SWEEP_OCCLUSION_NONE);
        cPS.setPlaneGenerationMode(PSL::PLANE_SWEEP_PLANEMODE_UNIFORM_DISPARITY);
        cPS.setMatchingCosts(PSL::PLANE_SWEEP_SAD);
        cPS.setSubPixelInterpolationMode(PSL::PLANE_SWEEP_SUB_PIXEL_INTERP_INVERSE);
        cPS.enableOutputBestDepth();
        cPS.enableColorMatching();
        cPS.enableOutputBestCosts(false);
        cPS.enableOuputUniquenessRatio(false);
        cPS.enableOutputCostVolume(false);
        cPS.enableSubPixel();

        int refid;

        for (int i = 0; i < adjacent_views_idxs.size(); i++)
        {
            if (i == 2)
            {
                std::string imageFileName = images_folder + image_files[reference_idx];
                //std::cout << imageFileName << std::endl;
                cv::Mat image = cv::imread(imageFileName);
                if (image.empty())
                {
                    PSL_THROW_EXCEPTION("Failed to load image")
                }
                refid = cPS.addImage(image, cameras[reference_idx]);

                //std::cout << "id:" << refid << std::endl;
            }
            int idx = adjacent_views_idxs[i];
            //std::cout << idx << std::endl;

            // load the image from disk
            std::string imageFileName = images_folder + image_files[idx];
            //std::cout << imageFileName << std::endl;
            cv::Mat image = cv::imread(imageFileName);
            if (image.empty())
            {
                PSL_THROW_EXCEPTION("Failed to load image")
            }

            if (cameras.count(i) != 1)
            {
                PSL_THROW_EXCEPTION("Camera for image was not loaded, something is wrong with the dataset")
            }
            int id = cPS.addImage(image, cameras[idx]);
        }

        {
            cPS.process(refid);
            PSL::DepthMap<float, double> dM;
            dM = cPS.getBestDepth();
            cv::Mat refImage = cPS.downloadImage(refid);

            makeOutputFolder("pinholeTestResults/colorSAD/NoOcclusionHandling/");
            cv::imwrite("pinholeTestResults/colorSAD/NoOcclusionHandling/refImg.png",refImage);
            dM.saveInvDepthAsColorImage("pinholeTestResults/colorSAD/NoOcclusionHandling/invDepthCol.png", minZ, maxZ);
            dM.saveAsDataFile("pinholeTestResults/colorSAD/NoOcclusionHandling/invDepthCol_data.bin");
        }

        {
            cPS.setOcclusionMode(PSL::PLANE_SWEEP_OCCLUSION_REF_SPLIT);
            cPS.process(refid);
            PSL::DepthMap<float, double> dM;
            dM = cPS.getBestDepth();
            cv::Mat refImage = cPS.downloadImage(refid);

            makeOutputFolder("pinholeTestResults/colorSAD/RefSplit/");
            cv::imwrite("pinholeTestResults/colorSAD/RefSplit/refImg.png",refImage);
            dM.saveInvDepthAsColorImage("pinholeTestResults/colorSAD/RefSplit/invDepthCol.png", minZ, maxZ);
            dM.saveAsDataFile("pinholeTestResults/colorSAD/RefSplit/invDepthCol_data.bin");
        }


        // now we add the remaining images and use best K occlusion handling
        //for (int i = 5; i < 25; i++)
        //{
        //    // load the image from disk
        //    std::string imageFileName = dataFolder + "/" + imageFileNames[i];
        //    cv::Mat image = cv::imread(imageFileName);
        //    if (image.empty())
        //    {
        //        PSL_THROW_EXCEPTION("Failed to load image")
        //    }

        //    if (cameras.count(i) != 1)
        //    {
        //        PSL_THROW_EXCEPTION("Camera for image was not loaded, something is wrong with the dataset")
        //    }

        //    int id = cPS.addImage(image, cameras[i]);
        //    if (i == 12)
        //    {
        //        refId = id;
        //    }
        //}

        //{
        //    cPS.setOcclusionMode(PSL::PLANE_SWEEP_OCCLUSION_BEST_K);
        //    cPS.setOcclusionBestK(5);
        //    cPS.process(refId);
        //    PSL::DepthMap<float, double> dM;
        //    dM = cPS.getBestDepth();
        //    cv::Mat refImage = cPS.downloadImage(refId);

        //    makeOutputFolder("pinholeTestResults/colorSAD/BestK/");
        //    cv::imwrite("pinholeTestResults/colorSAD/BestK/refImg.png",refImage);
        //    dM.saveInvDepthAsColorImage("pinholeTestResults/colorSAD/BestK/invDepthCol.png", minZ, maxZ);
        //    dM.saveAsDataFile("pinholeTestResults/colorSAD/BestK/invDepthCol_data.bin");
        //}
    }

    // First tests compute a depth map for the middle image of the first row
    {
        makeOutputFolder("pinholeTestResults/grayscaleSAD");
        makeOutputFolder("pinholeTestResults/grayscaleZNCC");

        PSL::CudaPlaneSweep cPS;
        cPS.setScale(0.75); // Scale the images down to 0.25 times the original side length
        cPS.setZRange(minZ, maxZ);
        cPS.setMatchWindowSize(27,27);
        cPS.setNumPlanes(256);
        cPS.setOcclusionMode(PSL::PLANE_SWEEP_OCCLUSION_NONE);
        cPS.setPlaneGenerationMode(PSL::PLANE_SWEEP_PLANEMODE_UNIFORM_DISPARITY);
        cPS.setSubPixelInterpolationMode(PSL::PLANE_SWEEP_SUB_PIXEL_INTERP_INVERSE);
        cPS.enableOutputBestDepth();
        cPS.enableColorMatching(false);
        cPS.enableOutputBestCosts(false);
        cPS.enableOuputUniquenessRatio(false);
        cPS.enableOutputCostVolume(false);
        cPS.enableSubPixel();

        int refid;

        for (int i = 0; i < adjacent_views_idxs.size(); i++)
        {
            if (i == 2)
            {
                std::string imageFileName = images_folder + image_files[reference_idx];
                //std::cout << imageFileName << std::endl;
                cv::Mat image = cv::imread(imageFileName);
                if (image.empty())
                {
                    PSL_THROW_EXCEPTION("Failed to load image")
                }
                refid = cPS.addImage(image, cameras[reference_idx]);
                //std::cout << "id:" << refid << std::endl;
            }
            int idx = adjacent_views_idxs[i];
            //std::cout << idx << std::endl;
            // load the image from disk
            std::string imageFileName = images_folder + image_files[idx];
            //std::cout << imageFileName << std::endl;
            cv::Mat image = cv::imread(imageFileName);
            if (image.empty())
            {
                PSL_THROW_EXCEPTION("Failed to load image")
            }

            if (cameras.count(i) != 1)
            {
                PSL_THROW_EXCEPTION("Camera for image was not loaded, something is wrong with the dataset")
            }
            int id = cPS.addImage(image, cameras[idx]);
        }

        {
            cPS.setMatchingCosts(PSL::PLANE_SWEEP_SAD);
            cPS.process(refid);
            PSL::DepthMap<float, double> dM;
            dM = cPS.getBestDepth();
            cv::Mat refImage = cPS.downloadImage(refid);

            makeOutputFolder("pinholeTestResults/grayscaleSAD/NoOcclusionHandling/");
            cv::imwrite("pinholeTestResults/grayscaleSAD/NoOcclusionHandling/refImg.png",refImage);
            dM.saveInvDepthAsColorImage("pinholeTestResults/grayscaleSAD/NoOcclusionHandling/invDepthCol.png", minZ, maxZ);
            dM.saveAsDataFile("pinholeTestResults/grayscaleSAD/NoOcclusionHandling/invDepthCol_data.bin");
        }

        {
            cPS.setMatchingCosts(PSL::PLANE_SWEEP_ZNCC);
            cPS.process(refid);
            PSL::DepthMap<float, double> dM;
            dM = cPS.getBestDepth();
            cv::Mat refImage = cPS.downloadImage(refid);

            makeOutputFolder("pinholeTestResults/grayscaleZNCC/NoOcclusionHandling/");
            cv::imwrite("pinholeTestResults/grayscaleZNCC/NoOcclusionHandling/refImg.png",refImage);
            dM.saveInvDepthAsColorImage("pinholeTestResults/grayscaleZNCC/NoOcclusionHandling/invDepthCol.png", minZ, maxZ);
            dM.saveAsDataFile("pinholeTestResults/grayscaleZNCC/NoOcclusionHandling/invDepthCol_data.bin");
        }

        cPS.setOcclusionMode(PSL::PLANE_SWEEP_OCCLUSION_REF_SPLIT);

        {
            cPS.setMatchingCosts(PSL::PLANE_SWEEP_SAD);
            cPS.process(refid);
            PSL::DepthMap<float, double> dM;
            dM = cPS.getBestDepth();
            cv::Mat refImage = cPS.downloadImage(refid);

            makeOutputFolder("pinholeTestResults/grayscaleSAD/RefSplit/");
            cv::imwrite("pinholeTestResults/grayscaleSAD/RefSplit/refImg.png",refImage);
            dM.saveInvDepthAsColorImage("pinholeTestResults/grayscaleSAD/RefSplit/invDepthCol.png", minZ, maxZ);
            dM.saveAsDataFile("pinholeTestResults/grayscaleSAD/RefSplit/invDepthCol_data.bin");
        }

        {
            cPS.setMatchingCosts(PSL::PLANE_SWEEP_ZNCC);
            cPS.process(refid);
            PSL::DepthMap<float, double> dM;
            dM = cPS.getBestDepth();
            cv::Mat refImage = cPS.downloadImage(refid);

            makeOutputFolder("pinholeTestResults/grayscaleZNCC/RefSplit/");
            cv::imwrite("pinholeTestResults/grayscaleZNCC/RefSplit/refImg.png",refImage);
            dM.saveInvDepthAsColorImage("pinholeTestResults/grayscaleZNCC/RefSplit/invDepthCol.png", minZ, maxZ);
            dM.saveAsDataFile("pinholeTestResults/grayscaleZNCC/RefSplit/invDepthCol_data.bin");
        }


        // now we add the remaining images and use best K occlusion handling
        //for (int i = 5; i < 25; i++)
        //{
        //    // load the image from disk
        //    std::string imageFileName = dataFolder + "/" + imageFileNames[i];
        //    cv::Mat image = cv::imread(imageFileName);
        //    if (image.empty())
        //    {
        //        PSL_THROW_EXCEPTION("Failed to load image")
        //    }

        //    if (cameras.count(i) != 1)
        //    {
        //        PSL_THROW_EXCEPTION("Camera for image was not loaded, something is wrong with the dataset")
        //    }

        //    int id = cPS.addImage(image, cameras[i]);
        //    if (i == 12)
        //    {
        //        refId = id;
        //    }
        //}

        //cPS.setOcclusionMode(PSL::PLANE_SWEEP_OCCLUSION_BEST_K);

        //{
        //    cPS.setMatchingCosts(PSL::PLANE_SWEEP_SAD);
        //    cPS.setOcclusionBestK(5);
        //    cPS.process(refId);
        //    PSL::DepthMap<float, double> dM;
        //    dM = cPS.getBestDepth();
        //    cv::Mat refImage = cPS.downloadImage(refId);

        //    makeOutputFolder("pinholeTestResults/grayscaleSAD/BestK/");
        //    cv::imwrite("pinholeTestResults/grayscaleSAD/BestK/refImg.png",refImage);
        //    dM.saveInvDepthAsColorImage("pinholeTestResults/grayscaleSAD/BestK/invDepthCol.png", minZ, maxZ);
        //    dM.saveAsDataFile("pinholeTestResults/grayscaleSAD/BestK/invDepthCol_data.bin");
        //}

        //{
        //    cPS.setMatchingCosts(PSL::PLANE_SWEEP_ZNCC);
        //    cPS.setOcclusionBestK(5);
        //    cPS.process(refId);
        //    PSL::DepthMap<float, double> dM;
        //    dM = cPS.getBestDepth();
        //    cv::Mat refImage = cPS.downloadImage(refId);

        //    makeOutputFolder("pinholeTestResults/grayscaleZNCC/BestK/");
        //    cv::imwrite("pinholeTestResults/grayscaleZNCC/BestK/refImg.png",refImage);
        //    dM.saveInvDepthAsColorImage("pinholeTestResults/grayscaleZNCC/BestK/invDepthCol.png", minZ, maxZ);
        //    dM.saveAsDataFile("pinholeTestResults/grayscaleZNCC/BestK/invDepthCol_data.bin");
        //}
    }
}
