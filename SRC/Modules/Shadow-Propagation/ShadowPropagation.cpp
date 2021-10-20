#include "ShadowPropagation.hpp"
#include <torch/cuda.h>

namespace ShadowPropagation {
    void ShadowPropagationEngine::ModuleMain(void) {
        while (! m_abort) {
            m_mutex.lock();
            if (! m_running) {
                m_mutex.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            if (m_unprocessedShadowMaps.empty()) {
                //There are no unprocessed shadow maps todeal with
                m_mutex.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            //If we get here, we have unprocessed shadow maps to deal with
            //TODO: *********************************************************************************
            //TODO: ***************************** Magic sauce goes here *****************************
            //TODO: *********************************************************************************
            //Compute new Time Available function by some means
            auto startClock = std::chrono::steady_clock::now();
            ShadowDetection::InstantaneousShadowMap & map(m_unprocessedShadowMaps[0]);


            //Update m_TimeAvail
            //Note: It might be a good idea to check if the buffer m_unprocessedShadowMaps is getting too large.
            //This is an indicator that we are falling behind real time. Even if we can't handle missing data gracefully we should
            //try to have a plan for this since if we fall behind real time our TA functions become worthless. Can we reset the model
            //and only process every other shadow map? This doubling the time step would be like shadows are moving twice as fast, but
            //a model trained for one time step may fork for another... just an idea.
//            torch::NoGradGuard no_grad;
            std::cerr << "CALLED!" << std::endl;
            m_TimeAvail.UL_LL = map.UL_LL;
            m_TimeAvail.UR_LL = map.UR_LL;
            m_TimeAvail.LL_LL = map.LL_LL;
            m_TimeAvail.LR_LL = map.LR_LL;

            // Resize 512 x 512 Mat from InstantaneousShadowMap into 64 x 64 for module input
            cv::Mat downsizedMap;
            cv::resize(map.Map, downsizedMap, cv::Size(64, 64), cv::INTER_AREA);
            // Model was trained using 0-1 float values instead of 0-255
            cv::Mat cvInputWithExcess;
            downsizedMap.convertTo(cvInputWithExcess, CV_32FC1, 1.f/255.0);
            // Need to remove the area outside of the fisheye lens
            // Using a mask to "black off" the outside area

            // Creating mask
            cv::Mat mask = cv::Mat::zeros(cv::Size(64, 64), CV_8UC1);

            // Creates inverted shadow map, then finds enclosing circle of largest contour to create mask
            cv::Mat invertedShadowMap;
            cv::threshold(downsizedMap, invertedShadowMap, 127, 255, cv::THRESH_BINARY_INV);
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(invertedShadowMap, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            cv::Point2f center;
            float radius;
            if (contours.size() > 0) {
                std::vector<cv::Point>& largestContour = contours[0];
                double largestContourArea = cv::contourArea(largestContour);
                for (int i = 0; i < contours.size(); i++) {
                    if (cv::contourArea(contours[i]) > largestContourArea) {
                        largestContour = contours[i];
                        largestContourArea = cv::contourArea(largestContour);
                    }
                }
                cv::minEnclosingCircle(largestContour, center, radius);
            }
            if (contours.size() <= 0 || radius < 29) {
                // Default circle + radius estimation so things don't fall apart
                radius = 32;
                center = cv::Point2f(33.7, 33.7);
            }
            cv::circle(mask, center, radius - 3, cv::Scalar(255), -1, 8, 0);
            cv::Mat cvInput;
            cvInputWithExcess.copyTo(cvInput, mask);

            if (!cvInput.isContinuous()) {
                std::cout << "Memory is NOT contiguous." << std::endl;
            }
            torch::Tensor inputTensorCompressed = torch::from_blob(cvInput.data, {64, 64}, torch::kFloat32).to(m_device);
            torch::Tensor inputTensor = inputTensorCompressed.unsqueeze(0).unsqueeze(0);
            if (at::isnan(inputTensor).any().item<bool>()) {
                std::cerr << "Input Tensor has a NaN value." << std::endl;
            }

            for (int i = 0; i < 64; i++) {
                for (int j = 0; j < 64; j++) {
                    if (inputTensor[0][0][i][j].item<float>() < 0 || inputTensor[0][0][i][j].item<float>() > 1 ||
                            std::isnan(inputTensor[0][0][i][j].item<float>())) {
                        std::cout << "Bad Input" << std::endl;
                        std::cout << inputTensor[0][0][i][j].item<float>() << std::endl;
                    }
                }
            }

//            allVectors.push_back(inputTensor);
            if (m_prevInputs.size() == TARGET_INPUT_LENGTH) {
                for (int i = 0; i < m_prevInputs.size(); i++) {
                    std::vector<torch::jit::IValue> inputs;
                    std::cout << m_prevInputs[i].sizes() << std::endl;
                    inputs.push_back(m_prevInputs[i]);
                    inputs.push_back((i == 0));
                    inputs.push_back(false);
//                if (at::isnan(inputs[0].toTensor()).all().item<bool>()) {
//                    std::cerr << "PREV INPUT IS A NAN TENSOR!" << std::endl;
//                }    std::vector<torch::Tensor> input;

                    auto result = m_module.forward(inputs);
//                    torch::Tensor outputTensor = result.toTuple()->elements()[2].toTensor();
//                    for (int i = 0; i < 64; i++) {
//                        for (int j = 0; j < 64; j++) {
//                            if (outputTensor[0][0][i][j].item<float>() < 0 || outputTensor[0][0][i][j].item<float>() > 1 ||
//                                std::isnan(outputTensor[0][0][i][j].item<float>())) {
//                                std::cout << "Bad Output Tensor from first foor loop" << std::endl;
//                                std::cout << outputTensor[0][0][i][j].item<float>() << std::endl;
//                            }
//                        }
//                    }

//                cv::Mat input(64, 64, CV_32FC1, m_prevInputs[i][0][0].data_ptr());
//                cv::imshow("PRE-INPUTs", input);
//                cv::waitKey(1);
                }
                cv::Mat localTimeAvailable(cv::Size(64, 64), CV_16UC1,
                                           cv::Scalar(std::numeric_limits<uint16_t>::max()));
                for (int t = 1; t <= TIME_HORIZON; t++) {
                    std::vector<torch::jit::IValue> inputs;
                    inputs.push_back(inputTensor);
                    inputs.push_back(false);
                    inputs.push_back(false);
//                if (at::isnan(inputs[0].toTensor()).all().item<bool>()) {
//                    std::cerr << "INPUT IS A NAN TENSOR!" << std::endl;
//                }
//                cv::Mat input(64, 64, CV_32FC1, inputTensor[0][0].data_ptr());
//                cv::imshow("INPUT", input);
//                cv::waitKey(1);
                    auto result = m_module.forward(inputs);
                    // decoder_input, decoder_hidden, output_image, _, _
                    torch::Tensor outputTensor = result.toTensor();
//                    torch::Tensor outputTensor = result.toTuple()->elements()[2].toTensor();
                    std::cout << "Output Tensor Dims: " << outputTensor.sizes() << std::endl;
                    if (at::isnan(outputTensor).any().item<bool>()) {
                        if (!isSaved && counter == 0) {
                            isSaved = true;
                            std::cout << inputTensor << std::endl;
                            std::cout << outputTensor << std::endl;
                            torch::save(inputTensor, "x.pt");
                        }
                        counter++;
                        std::cout << " ANOMALY ALERT!" << std::endl;
                    }
                    auto accessor = outputTensor.accessor<float, 4>();
                    for (int i = 0; i < 64; i++) {
                        for (int j = 0; j < 64; j++) {
                            if (isnan(accessor[0][0][i][j])) {
                                accessor[0][0][i][j] = 0;
                            }
//                        if (accessor[0][0][i][j] < 0) {
//                            std::cout << "NEGATIVE" << std::endl;
//                        }
                        }
                    }
//                cv::Mat output(64, 64, CV_32FC1, outputTensor[0][0].data_ptr());
//                cv::imshow("OUTPUT!!", output);
//                cv::waitKey(1);
                    // Iterates over output tensor and then updates localTimeAvailable accordingly
                    for (int i = 0; i < 64; i++) {
                        for (int j = 0; j < 64; j++) {
                            if (outputTensor[0][0][i][j].item<float>() > OUTPUT_THRESHOLD &&
                                localTimeAvailable.at<uint16_t>(i, j) > t) {
                                localTimeAvailable.at<uint16_t>(i, j) = t;
                            }
                        }
                    }
                    inputTensor = outputTensor.clone();
                }
                // Scales up localTimeAvailable
                cv::resize(localTimeAvailable, m_TimeAvail.TimeAvailable, cv::Size(512, 512), cv::INTER_LINEAR);
                m_TimeAvail.Timestamp = map.Timestamp;
                auto endClock = std::chrono::steady_clock::now();
                for (auto const & kv : m_callbacks)
                    kv.second(m_TimeAvail);
            }
            m_prevInputs.push_back(inputTensor);
            if (m_prevInputs.size() > TARGET_INPUT_LENGTH) {
                m_prevInputs.pop_front();
            }
//            numImagesProcessed++;
//            numMicroseconds += std::chrono::duration_cast<std::chrono::milliseconds>(endClock - startClock).count();
            m_unprocessedShadowMaps.erase(m_unprocessedShadowMaps.begin()); //Will cause re-allocation but the buffer is small so don't worry about it
            //TODO: *********************************************************************************
            //TODO: *********************************************************************************
            //TODO: *********************************************************************************

            //Unlock but don't snooze if we actually did work
            m_mutex.unlock();
        }
    }
}