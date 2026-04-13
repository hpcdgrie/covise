#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

struct WebcamDevice
{
    int index = -1;
    std::string name;
    std::string description;
};

static std::string normalize(const std::string &s)
{
    std::string out;
    out.reserve(s.size());
    for (unsigned char c : s)
    {
        if (std::isalnum(c) || std::isspace(c))
            out.push_back(static_cast<char>(std::tolower(c)));
        else
            out.push_back(' ');
    }
    return out;
}

static std::vector<std::string> splitWords(const std::string &s)
{
    std::vector<std::string> words;
    std::istringstream iss(s);
    std::string w;
    while (iss >> w)
        words.push_back(w);
    return words;
}

static int matchScore(const std::string &query, const WebcamDevice &dev)
{
    const std::string q = normalize(query);
    const std::string hay = normalize(dev.name + " " + dev.description);
    int score = 0;

    if (hay == q)
        score += 200;
    if (!q.empty() && hay.find(q) != std::string::npos)
        score += 120;

    const auto qWords = splitWords(q);
    for (const auto &w : qWords)
    {
        if (!w.empty() && hay.find(w) != std::string::npos)
            score += 20;
    }

    return score;
}

static std::vector<WebcamDevice> enumerateWindowsWebcams()
{
    // No explicit sort here: keep PowerShell/native enumeration order.
    const char *cmd =
        "powershell -NoProfile -Command \""
        "$devices = Get-CimInstance Win32_PnPEntity | "
        "Where-Object { $_.Status -eq 'OK' -and ($_.PNPClass -eq 'Camera' -or $_.PNPClass -eq 'Image') }; "
        "$devices | Select-Object Name,Description | "
        "ForEach-Object { '{0}`t{1}' -f $_.Name, $_.Description }"
        "\"";

    std::vector<WebcamDevice> devices;
    std::unique_ptr<FILE, decltype(&_pclose)> pipe(_popen(cmd, "r"), _pclose);
    if (!pipe)
        return devices;

    char buffer[4096];
    int idx = 0;
    while (fgets(buffer, sizeof(buffer), pipe.get()))
    {
        std::string line(buffer);
        while (!line.empty() && (line.back() == '\n' || line.back() == '\r'))
            line.pop_back();

        const auto tabPos = line.find('\t');
        WebcamDevice d;
        d.index = idx++;
        if (tabPos == std::string::npos)
        {
            d.name = line;
            d.description = line;
        }
        else
        {
            d.name = line.substr(0, tabPos);
            d.description = line.substr(tabPos + 1);
        }
        devices.push_back(std::move(d));
    }
    return devices;
}

static bool isFunctionalOpenCvCamera(int index)
{
    cv::VideoCapture cap;
    for (int mode: {cv::CAP_V4L2, cv::CAP_ANY})
    {
        if (cap.open(index, mode))
            break;

    }
    if(!cap.isOpened())
        return false;
    std::cerr << "capture device: device " << index << " opened successfully with OpenCV backend " << cap.getBackendName() << std::endl;
    cv::Mat frame;
    for (int i = 0; i < 10; ++i)
    {
        if (cap.read(frame) && !frame.empty())
        {
            std::string filename = "webcam_" + std::to_string(index) + ".png";
            cv::imwrite(filename, frame);
            std::cerr << "Wrote frame: " << filename << std::endl;
            return true;
        }
    }
    return false;
}


int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: getWebcamId <device-description-string>" << std::endl;
        return 1;
    }

    std::string query = argv[1];
    for (int i = 2; i < argc; ++i)
    {
        query += " ";
        query += argv[i];
    }

    auto devices = enumerateWindowsWebcams();
    if (devices.empty())
    {
        std::cerr << "Error: no webcam devices found" << std::endl;
        return 2;
    }

    std::vector<WebcamDevice> functional;
    std::vector<bool> functionalStates;
    functional.reserve(devices.size());
    functionalStates.reserve(devices.size());
    putenv(strdup("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS=0"));
    for (const auto &d : devices)
    {
        if (isFunctionalOpenCvCamera(d.index))
        {
            std::cerr << "Found functional OpenCV webcam: index=" << d.index << ", name='" << d.name
                      << "', description='" << d.description << "'" << std::endl;
            functional.push_back(d);
            functionalStates.push_back(true);
        }
        else
        {
            std::cerr << "Webcam not functional with OpenCV: index=" << d.index << ", name='" << d.name
                      << "', description='" << d.description << "'" << std::endl;
            functionalStates.push_back(false);
        }
    }
    std::cerr << std::endl;
    for (size_t i = 0; i < devices.size(); i++)
    {
        const auto &d = devices[i];
        std::cerr << "Found webcam: index=" << d.index << ", name='" << d.name
                << "', description='" << d.description << "' works=" << functionalStates[i] << std::endl;
    }


    if (functional.empty())
    {
        std::cerr << "Error: no functional OpenCV webcam found" << std::endl;
        return 3;
    }

    const auto best = std::max_element(functional.begin(), functional.end(),
        [&query](const WebcamDevice &a, const WebcamDevice &b)
        {
            return matchScore(query, a) < matchScore(query, b);
        });

    const int score = matchScore(query, *best);
    if (score <= 0)
    {
        std::cerr << "Error: no matching webcam description for query: " << query << std::endl;
        return 4;
    }

    // Print only the selected OpenCV device index for easy scripting.
    std::cout << best->index << std::endl;
    return 0;
}