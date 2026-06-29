#include <torch/torch.h>
#include <torch/fft.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <iostream>
#include <string>
#include <chrono>
#include <cmath>



// Helper function to extract 2D patches from a padded image
std::vector<cv::Mat> getPatches2D(const cv::Mat& padded, const cv::Size& patch_size) {
    std::vector<cv::Mat> patches;
    int rows = padded.rows - patch_size.height + 1;
    int cols = padded.cols - patch_size.width + 1;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            cv::Rect roi(j, i, patch_size.width, patch_size.height);
            patches.push_back(padded(roi).clone());
        }
    }
    return patches;
}

// Main function equivalent to patch_normalise_pad
cv::Mat patchNormalisePad(const cv::Mat& image, const cv::Size& patch_size) {
    CV_Assert(patch_size.width % 2 == 1 && patch_size.height % 2 == 1);
    int half_h = (patch_size.height - 1) / 2;
    int half_w = (patch_size.width - 1) / 2;

    // Convert to float32
    cv::Mat float_img;
    image.convertTo(float_img, CV_32F);

    // Pad the image with NaN
    cv::Mat padded(image.rows + 2 * half_h, image.cols + 2 * half_w, CV_32F, cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
    float_img.copyTo(padded(cv::Rect(half_w, half_h, image.cols, image.rows)));

    // Extract patches
    auto patches = getPatches2D(padded, patch_size);
    int nrows = image.rows;
    int ncols = image.cols;

    // Compute mean and stddev
    cv::Mat mean_map = cv::Mat::zeros(nrows, ncols, CV_32F);
    cv::Mat std_map = cv::Mat::zeros(nrows, ncols, CV_32F);

    for (int i = 0; i < nrows * ncols; ++i) {
        const cv::Mat& patch = patches[i];
        cv::Scalar mean, stddev;
        cv::meanStdDev(patch, mean, stddev, ~cv::Mat(patch != patch)); // Mask NaNs
        mean_map.at<float>(i / ncols, i % ncols) = mean[0];
        std_map.at<float>(i / ncols, i % ncols) = stddev[0];
    }

    // Normalize
    cv::Mat out = (float_img - mean_map) / std_map;

    // Handle NaNs and clamp values
    for (int i = 0; i < out.rows; ++i) {
        for (int j = 0; j < out.cols; ++j) {
            float& val = out.at<float>(i, j);
            if (std::isnan(val))
                val = 127.0f;
            else if (val < -1.0f)
                val = 0.0;
            else if (val > 1.0f)
                val = 255.0f;
        }
    }

    out.convertTo(out, CV_8U);
    return out;
}


// Function to perform cross-correlation between two images in frequency domain
torch::Tensor fftconvolve_torch_cpp(const torch::Tensor& a, const torch::Tensor& b, const std::string& mode = "same") {
    torch::Tensor result;    

    // Ensure tensors are float32 and on the same device
    auto a_f = a.detach().to(torch::kFloat32);
    auto b_f = b.detach().to(torch::kFloat32);

    auto a_shape = a.sizes();
    auto b_shape = b.sizes();

    std::vector<int64_t> out_size;
    if (mode == "full") {
        for (size_t i = 0; i < a.dim(); ++i) {
            out_size.push_back(a_shape[i] + b_shape[i] - 1);
        }
    } else if (mode == "same") {
        out_size = a_shape.vec();
    } else if (mode == "valid") {
        for (size_t i = 0; i < a.dim(); ++i) {
            out_size.push_back(a_shape[i] - b_shape[i] + 1);
        }
    } else {
        throw std::invalid_argument("mode must be 'full', 'same', or 'valid'");
    }

    // For simplicity, assume input shapes are same or compatible
    std::vector<int64_t> fft_size(a_shape.begin(), a_shape.end());

    // Perform FFT (use rfft2 if both a and b are real)
    auto a_fft = torch::fft::rfft2(a_f, fft_size);
    auto b_fft = torch::fft::rfft2(b_f, fft_size);

    // Multiply in frequency domain
    auto result_fft = a_fft * b_fft;

    // Inverse FFT
    result = torch::fft::irfft2(result_fft, fft_size);

    // Slicing for different modes
    if (mode == "same" || mode == "valid") {
        std::vector<torch::indexing::TensorIndex> slices;
        for (size_t i = 0; i < out_size.size(); ++i) {
            int64_t start, end;
            if (mode == "same") {
                start = (fft_size[i] - out_size[i]) / 2;
                end = start + out_size[i];
            } else { // valid
                start = fft_size[i] - out_size[i];
                end = start + out_size[i];
            }
            slices.push_back(torch::indexing::Slice(start, end));
        }
        result = result.index(slices);
    }
    return result;
}

cv::Mat reduceWidthBySumming8(const cv::Mat& input) {
    int rows = input.rows;
    int cols = input.cols;

    // Output image will have the same number of rows but 1/8 the number of columns
    int reducedCols = cols / 8;
    cv::Mat output(rows, reducedCols, CV_32FC1, cv::Scalar(0));

    for (int i = 0; i < rows; ++i) {
        const float* inputRow = input.ptr<float>(i);
        float* outputRow = output.ptr<float>(i);

        for (int j = 0; j < reducedCols; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < 8; ++k) {
                sum += inputRow[j * 8 + k];
            }
            outputRow[j] = sum;
        }
    }

    return output;
}

cv::Mat reduceHeightBySumming8(const cv::Mat& input) {
    int rows = input.rows;
    int cols = input.cols;

    // Output image will have 1/8 the number of rows but the same number of columns
    int reducedRows = rows / 8;
    cv::Mat output(reducedRows, cols, CV_32FC1, cv::Scalar(0));

    for (int i = 0; i < reducedRows; ++i) {
        float* outputRow = output.ptr<float>(i);
        for (int k = 0; k < 8; ++k) {
            const float* inputRow = input.ptr<float>(i * 8 + k);
            for (int j = 0; j < cols; ++j) {
                outputRow[j] += inputRow[j];
            }
        }
    }

    return output;
}

cv::Mat visualize(const cv::Mat& qry, const cv::Mat& match, int match_y) {
    cv::Mat qry_color, match_color;

    // Convert grayscale to BGR if needed
    if (qry.channels() == 1) {
        cv::cvtColor(qry, qry_color, cv::COLOR_GRAY2BGR);
        cv::cvtColor(match, match_color, cv::COLOR_GRAY2BGR);
    } else {
        qry_color = qry.clone();
        match_color = match.clone();
    }

    // Draw green vertical line in query image (center)
    int qry_center_x = qry.cols / 2;
    cv::line(qry_color, cv::Point(qry_center_x, 0), cv::Point(qry_center_x, qry.rows), cv::Scalar(0, 255, 0), 2);

    // Draw green vertical line in match image at match_y
    cv::line(match_color, cv::Point(match_y, 0), cv::Point(match_y, match.rows), cv::Scalar(0, 255, 0), 2);

    // Concatenate vertically
    cv::Mat combined;
    cv::vconcat(qry_color, match_color, combined);

    return combined;
}


std::tuple<std::vector<int>, std::vector<float>, cv::Mat> concat_fft_match_images_debug(
    cv::Mat& image_raw, cv::Mat& template_image_raw, int num_imgs, int centre_image_index, int subsampling = 1
) {

    cv::Mat image = image_raw; // reduceWidthBySumming8(image_raw);
    int img_width = image.cols / num_imgs;

    cv::Mat qry_img = template_image_raw; //reduceWidthBySumming8(template_image_raw);
    cv::Mat template_image_flipped;
    cv::flip(template_image_raw, template_image_flipped, -1);

    torch::Tensor corr_tensor = fftconvolve_torch_cpp(
        torch::from_blob(image.data, {image.rows, image.cols}),
        torch::from_blob(qry_img.data, {qry_img.rows, qry_img.cols}),
        "valid"
    );

    cv::Mat corr(corr_tensor.size(0), corr_tensor.size(1), CV_32F, corr_tensor.data_ptr());
    corr = corr.colRange(0, corr.cols - 1);

    std::vector<float> cor_arr;
    std::vector<int> offset_arr;
    cv::Mat debug_image;

    for (int i = 0; i < num_imgs; ++i) {
        int start = i * img_width;
        int end = start + (img_width / 2);

        // cv::Mat cor = corr.colRange(start + (img_width / 8), end - (img_width / 8));
        cv::Mat cor = corr.colRange(start, end);

        cv::Point max_loc;
        cv::minMaxLoc(cor, nullptr, nullptr, nullptr, &max_loc);
        // int max_y = max_loc.x + (img_width / 8); // Adjust for the padding
        int max_y = max_loc.x;

        cor_arr.push_back(cor.at<float>(max_y));
        int offset = max_y - (img_width / 4);
        offset_arr.push_back(offset);

        if (offset < -img_width / 4 || offset > img_width / 4) {
            std::cerr << "Offset out of bounds: " << offset << std::endl;
            continue;
        }

        // if (i == centre_image_index) {
        //     int multiplier = 8;
        //     int start_raw = img_width*multiplier*i;
        //     cv::Mat raw_image = image_raw(cv::Rect(start_raw + (img_width*multiplier) / 4, 0, (img_width*multiplier) / 2, image_raw.rows)).clone();

        //     cv::Mat temp_disp, raw_disp;
        //     template_image_flipped.convertTo(temp_disp, CV_8U, 1.0, 64);
        //     raw_image.convertTo(raw_disp, CV_8U, 1.0, 64);

        //     debug_image = visualize(temp_disp, raw_disp, max_y*multiplier);
        // }
    }


    cv::Point best_match_loc;
    cv::minMaxLoc(cor_arr, nullptr, nullptr, nullptr, &best_match_loc);
    int best_match_index = best_match_loc.x;
    int best_match_offset = offset_arr[best_match_index];
    int multiplier = 1;
    int start_raw = img_width * multiplier * best_match_index;
    cv::Mat best_match_img = image_raw(cv::Rect(start_raw + (img_width * multiplier) / 4, 0, (img_width * multiplier) / 2, image_raw.rows)).clone();
    cv::Mat temp_disp, raw_disp;
    template_image_flipped.convertTo(temp_disp, CV_8U, 1.0, 64);
    best_match_img.convertTo(raw_disp, CV_8U, 1.0, 64);
    debug_image = visualize(temp_disp, raw_disp, (best_match_offset + img_width / 4)*multiplier);

    return {offset_arr, cor_arr, debug_image};
}



// int main (int argc, char* argv[]) {

//     if (argc < 3) {
//         std::cerr << "Usage: " << argv[0] << " <image_path> <template_image_path>" << std::endl;
//         return 1;
//     }

//     std::vector<cv::Mat> images;

//     for (const auto& entry : std::filesystem::directory_iterator(argv[1])) {
//         if (entry.is_regular_file()) {
//             std::string file_path = entry.path().string();
//             cv::Mat img = cv::imread(file_path, cv::IMREAD_GRAYSCALE);
//             cv::resize(img, img, cv::Size(), 0.25, 0.25, cv::INTER_NEAREST);
//             cv::flip(img, img, -1);
//             cv::threshold(img, img, 50, 255, cv::THRESH_BINARY);
//             cv::Mat img_padded;
//             cv::copyMakeBorder(img, img_padded, 0, 0, img.cols/2, img.cols/2, cv::BORDER_CONSTANT, cv::Scalar(0));
//             img_padded.convertTo(img_padded, CV_32F);
//             img_padded -= 64;
//             images.push_back(img_padded);
//         }
//     }

//     cv::Mat qry_img = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
//     cv::resize(qry_img, qry_img, cv::Size(), 0.25, 0.25, cv::INTER_NEAREST);
//     cv::threshold(qry_img, qry_img, 50, 255, cv::THRESH_BINARY);
//     qry_img.convertTo(qry_img, CV_32F);
//     qry_img -= 64;


//     auto start = std::chrono::high_resolution_clock::now();
//     int count = 0;
//     for (int iter = 0; iter < 11; iter++) {
//         cv::Mat ref_imgs_padded;
//         cv::hconcat(images, ref_imgs_padded);

//         auto [offsets, correlations, debug_img] = concat_fft_match_images_debug(
//             ref_imgs_padded, qry_img, images.size(), iter, 1
//         );

//         cv::imshow("Reference Images", debug_img);
//         cv::waitKey(0);
//         count++;
//     }
//     auto end = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double> elapsed = end - start;
//     std::cout << "Function took " << elapsed.count() << " seconds.\n";
//     std::cout << "Number of iterations: " << count << std::endl;
//     std::cout << "Average Throughput: " << (count / elapsed.count()) << " iterations/sec" << std::endl;

//     return 0;
// }


int main (int argc, char* argv[]) {

    std::vector<std::filesystem::directory_entry> ref_entries;

    // Collect all regular files
    for (const auto& entry : std::filesystem::directory_iterator(argv[1])) {
        if (entry.is_regular_file())
            ref_entries.push_back(entry);
    }

    // Sort files lexicographically by filename
    std::sort(ref_entries.begin(), ref_entries.end(),
              [](const std::filesystem::directory_entry& a, const std::filesystem::directory_entry& b) {
                  return a.path().filename().string() < b.path().filename().string();
              });

    std::vector<cv::Mat> ref_images;
    for (const auto& entry : ref_entries) {
        std::string file_path = entry.path().string();
        cv::Mat img = cv::imread(file_path, cv::IMREAD_GRAYSCALE);
        // cv::resize(img, img, cv::Size(115, 40));
        cv::resize(img, img, cv::Size(), 0.125, 0.125, cv::INTER_NEAREST);
        cv::flip(img, img, -1);
        cv::threshold(img, img, 50, 255, cv::THRESH_BINARY);
        cv::Mat img_padded;
        cv::copyMakeBorder(img, img_padded, 0, 0, img.cols/2, img.cols/2, cv::BORDER_CONSTANT, cv::Scalar(0));
        img_padded.convertTo(img_padded, CV_32F);
        img_padded -= 64;
        ref_images.push_back(img_padded);
    }

    std::vector<std::filesystem::directory_entry> qry_entries;

    // Collect all regular files
    for (const auto& entry : std::filesystem::directory_iterator(argv[2])) {
        if (entry.is_regular_file())
            qry_entries.push_back(entry);
    }

    // Sort files lexicographically by filename
    std::sort(qry_entries.begin(), qry_entries.end(),
              [](const std::filesystem::directory_entry& a, const std::filesystem::directory_entry& b) {
                  return a.path().filename().string() < b.path().filename().string();
              });

    std::vector<cv::Mat> qry_images;
    for (const auto& entry : qry_entries) {
        std::string file_path = entry.path().string();
        cv::Mat img = cv::imread(file_path, cv::IMREAD_GRAYSCALE);
        // cv::resize(img, img, cv::Size(115, 40));
        cv::resize(img, img, cv::Size(), 0.125, 0.125, cv::INTER_NEAREST);
        cv::threshold(img, img, 50, 255, cv::THRESH_BINARY);
        img.convertTo(img, CV_32F);
        img -= 64;
        qry_images.push_back(img);
    }

    cv::Mat ref;
    cv::hconcat(ref_images, ref);

    int iter = 0;
    for (cv::Mat img : qry_images) {
        auto start = std::chrono::high_resolution_clock::now();

        auto [offsets, correlations, debug_img] = concat_fft_match_images_debug(
            ref, img, ref_images.size(), 0, 1
        );

        // cv::imshow("Reference Images", debug_img);
        // cv::waitKey(0);

        cv::imwrite("/home/gokulbnr/vpr_compressed/" + std::to_string(iter) + ".png", debug_img);

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::cout << "Function took " << elapsed.count() << " seconds.\n";

        iter++;
    }

    return 0;
}


// int main (int argc, char* argv[]) {

//     if (argc < 3) {
//         std::cerr << "Usage: " << argv[0] << " <image_path> <template_image_path>" << std::endl;
//         return 1;
//     }

//     std::vector<cv::Mat> images;
//     for (const auto& entry : std::filesystem::directory_iterator(argv[1])) {
//         if (entry.is_regular_file()) {
//             std::string file_path = entry.path().string();
//             cv::Mat img = cv::imread(file_path, cv::IMREAD_GRAYSCALE);
//             // cv::resize(img, img, cv::Size(), 0.25, 0.25, cv::INTER_NEAREST);
//             cv::flip(img, img, -1);
//             /* The above code is converting an image represented by the variable `img` to a new data
//             type using the `convertTo` function in OpenCV. Specifically, it is converting the image
//             to a signed 8-bit integer data type (CV_8S). This means that the pixel values in the
//             image will be represented as signed integers ranging from -128 to 127. */
//             // cv::threshold(img, img, 50, 255, cv::THRESH_BINARY);
//             cv::Mat img_padded;
//             cv::copyMakeBorder(img, img_padded, 0, 0, img.cols/2, img.cols/2, cv::BORDER_CONSTANT, cv::Scalar(0));
//             img_padded.convertTo(img_padded, CV_32F);
//             img_padded -= 64;
//             images.push_back(img_padded);
//         }
//     }

//     cv::Mat qry_img = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
//     // cv::resize(qry_img, qry_img, cv::Size(), 0.25, 0.25, cv::INTER_NEAREST);
//     // cv::threshold(qry_img, qry_img, 50, 255, cv::THRESH_BINARY);
//     qry_img.convertTo(qry_img, CV_32F);
//     qry_img -= 64;

//     cv::Mat template_image_flipped;
//     cv::flip(qry_img, template_image_flipped, -1);


//     for (int iter = 0; iter < 5; iter++) {
//         auto start = std::chrono::high_resolution_clock::now();

//         torch::Tensor corr_tensor = fftconvolve_torch_cpp(
//             torch::from_blob(images[iter].data, {images[iter].rows, images[iter].cols}),
//             torch::from_blob(qry_img.data, {qry_img.rows, qry_img.cols}),
//             "valid"
//         );

//         cv::Mat corr(corr_tensor.size(0), corr_tensor.size(1), CV_32F, corr_tensor.data_ptr());
//         corr = corr.colRange(0, corr.cols - 1);

//         std::cout << "corr size: " << corr.size() << std::endl;

//         cv::Point max_loc;
//         cv::minMaxLoc(corr, nullptr, nullptr, nullptr, &max_loc);
//         int max_y = max_loc.x;
//         int img_width = images[iter].cols;
//         std::cout << "Offset: " << max_y - (img_width / 4) << std::endl;

//         // Visualize the result
//         cv::Mat raw_image = images[iter](cv::Rect(img_width / 4, 0, img_width / 2, images[iter].rows)).clone();

//         cv::Mat temp_disp, raw_disp;
//         template_image_flipped.convertTo(temp_disp, CV_8U, 1.0, 64);
//         raw_image.convertTo(raw_disp, CV_8U, 1.0, 64);

//         cv::Mat debug_image = visualize(temp_disp, raw_disp, max_y);

//         auto end = std::chrono::high_resolution_clock::now();
//         std::chrono::duration<double> elapsed = end - start;
//         std::cout << "Function took " << elapsed.count() << " seconds.\n";

//         cv::imshow("Reference Images", debug_image);
//         cv::waitKey(0);
//     }

//     return 0;
// }


// int main(int argc, char* argv[]) {
//     // torch::set_num_threads(4);

//     if (argc < 3) {
//         std::cerr << "Usage: " << argv[0] << " <image1> <image2>" << std::endl;
//         return 1;
//     }

//     std::string image_path1 = argv[1];
//     std::string image_path2 = argv[2];

//     // Load images in grayscale
//     cv::Mat img1 = cv::imread(image_path1, cv::IMREAD_GRAYSCALE);
//     cv::Mat img2 = cv::imread(image_path2, cv::IMREAD_GRAYSCALE);

//     auto start = std::chrono::high_resolution_clock::now();

//     // cv::resize(img1, img1, cv::Size(), 0.25, 0.25, cv::INTER_NEAREST);
//     // cv::resize(img2, img2, cv::Size(), 0.25, 0.25, cv::INTER_NEAREST);

//     cv::threshold(img1, img1, 50, 255, cv::THRESH_BINARY);
//     cv::threshold(img2, img2, 50, 255, cv::THRESH_BINARY);

//     cv::flip(img2, img2, -1);

//     // Convert OpenCV Mat to Torch Tensor (H, W) → (1, H, W)
//     torch::Tensor tensor1 = torch::from_blob(img1.data, {img1.rows, img1.cols}, torch::kUInt8);
//     torch::Tensor tensor2 = torch::from_blob(img2.data, {img2.rows, img2.cols}, torch::kUInt8);

//     for(int i = 0; i < 1000; i++) fftconvolve_torch_cpp(tensor1, tensor2, "valid");

//     // Call FFT convolution
//     torch::Tensor result = fftconvolve_torch_cpp(tensor1, tensor2, "valid");

//     // std::cout << "result shape: " << result.sizes() << std::endl;
//     // std::cout << "result: " << result << std::endl;

//     auto end = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double> elapsed = end - start;
//     std::cout << "Function took " << elapsed.count() << " seconds.\n";

//     // Convert result to OpenCV Mat
//     result = result.squeeze();  // (H, W)
//     auto max_result = result.flatten().max(0);

//     // Find the (x, y) position of the maximum value
//     torch::Tensor max_val_tensor = std::get<0>(max_result);
//     torch::Tensor max_idx_tensor = std::get<1>(max_result);
//     int max_index = max_idx_tensor.item<int>();

//     int height = result.size(0);
//     int width = result.size(1);

//     // Convert flat index to 2D coordinates
//     int max_y = max_index % width;
//     int max_x = max_index / width;

//     cv::Mat result_img(result.size(0), result.size(1), CV_8U, result.data_ptr());

//     // Draw vertical line at column = max_y
//     cv::line(result_img, cv::Point(max_y, 0), cv::Point(max_y, height - 1), cv::Scalar(255), 1); // White line    

//     // Display result
//     cv::imshow("FFT Convolution Result", result_img);
//     cv::waitKey(0);

//     return 0;
// }