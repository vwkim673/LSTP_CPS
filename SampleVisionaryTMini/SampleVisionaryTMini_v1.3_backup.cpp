#define _CRT_SECURE_NO_WARNINGS
#define BOOST_ALL_NO_LIB
#define _USE_MATH_DEFINES
#define WIN32_LEAN_AND_MEAN

#pragma warning(disable: 4819 4996 4477 6258 4100)
#ifndef BOOST_USE_WINDOWS_H
#define BOOST_USE_WINDOWS_H
#endif

#define NOMINMAX
#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

#include <cstdio>
#include <iostream>
#include <memory>
#include <sstream>
#include <fstream>
#include <iterator>
#include <bitset>
#include <chrono>
#include <future>
#include <thread>
#include <filesystem>
#include <direct.h>
#include <vector>
#include <numeric>
#include <condition_variable>
#include <atomic>
#include <signal.h>
#include <queue>
#include <ctime>
#include <iomanip>

#ifndef _WINDOW_SOCKET
#define _WINDOW_SOCKET
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
#include <synchapi.h>
#endif

#include "VisionaryControl.h"
#include "CoLaParameterReader.h"
#include "CoLaParameterWriter.h"
#include "VisionaryTMiniData.h"    // Header specific for the Time of Flight data
#include "VisionaryDataStream.h"
#include "PointXYZ.h"
#include "PointCloudPlyWriter.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/videoio.hpp>

#include "miniINI.h"

#include <boost/filesystem.hpp>

#ifndef PCL_COMMON_H
#define PCL_COMMON_H
#include <pcl/pcl_base.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/console/parse.h>
#include <pcl/common/random.h>
#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>
#include <pcl/common/intersections.h>
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>
#include <pcl/common/norms.h>
#include <pcl/common/time.h>
#include <pcl/common/distances.h>

#include <boost/thread.hpp>

// filter include
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

// segmentation include
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// region growing
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>

//sample consensus for table calibrations
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_line.h>

#endif

#include <open3d/Open3D.h>

#include "GlobalVar.h"
#include "DataStack.h"

#pragma comment(lib, "Dbghelp.lib")
#include <DbgHelp.h>

#define DEFAULT_RECVLEN 50
#define DEFAULT_SENDLEN 50

namespace fs = std::filesystem;

/*
* Update: 2025.08.19
* Update Log:
*	Added CLPS OK (chassis-container separation detection logic)
*/

const std::string program_version = "1.3";

struct bbx
{
	int label = 0;
	float x = 0;
	float y = 0;
	float center_x = 0;
	float center_y = 0;
	float w = 0;
	float h = 0;
	float prob = 0;
};

std::string app_path;

#pragma region INI Variables

std::string sensor_ip;
std::string sensor_port;

std::string SOCKET_IP = "127.0.0.1";
int SOCKET_PORT;

const int sendLen = 50;
const int recvLen = 50;

bool MODE_DEBUG = true;

std::string DEBUG_SAMPLE_JOB = "";

bool DEBUG_WITH_FILES = false;
std::string DEBUG_PATH;
std::string DEBUG_SENSOR_POSITION = "REAR_LEFT";

bool DEBUG_BATCH_JOB = false;
std::string DEBUG_BATCH_ROOT_DIR = "";
std::string DEBUG_BATCH_SAVE_DIR = "";

bool DEBUG_CONVERT_PCL_RANGE = false;

bool SEQUENTIAL_PROCESSING = false;
bool ENALBE_RESTART_APPLICATION = false;

std::string CURRENT_SENSOR_POSITION = "REAR_LEFT";

// ==== 로깅 정책 파라미터 ====
int  FRAMES_PER_BUCKET;// = ini.geti("CPS.FRAMES_PER_BUCKET", 5);     // 1m 구간당 저장 개수
int  STABLE_DELTA_MM;// = ini.geti("CPS.STABLE_DELTA_MM", 20);      // 정차 판정 이동량
int  STABLE_FRAMES;// = ini.geti("CPS.STABLE_FRAMES", 8);         // 정차 판정 프레임 수
int  BUCKET_HYST_MM;// = ini.geti("CPS.BUCKET_HYST_MM", 50);       // 버킷 전환 (mm)
bool ENABLE_SAVE_LOG;// = ini.getb("SYSTEM.ENABLE_SAVE_LOG_BIT", true);
bool DEFAULT_ENABLE;// = ini.getb("CPS.DEFAULT_ENABLE", true);      // PLC 신호 없을 때 기본값

int TARGET_STOP_POSITION_MM = 1900;

bool ENABLE_SAVE_ALL = false;

std::vector<std::string> IP_ADDRESSES = {};
#pragma endregion

SOCKET connectSocket_ = NULL;
unsigned int heartbeat_, prev_heartbeat_;
int visionary_version_[4]{ 0 };
char block_n_[5]{};
char* block_;
int tzms_version_;

char* sendBuffer_;
char sendBuf_[DEFAULT_SENDLEN] = {};

char* recvBuffer_;
char recvBuf_[DEFAULT_RECVLEN] = {};

auto sck_connected = false;
//auto save_frame = false;

bool blnDataSave;
std::string SaveDir;
std::string SaveImageDir;
std::string SaveDepthDir;

//Receive from PLC data
bool CPS_Lane_Enable[6]{ false, false, false, false, false, false };
short Active_Job_Lane = 0;

//Send to PLC data
bool CPS_Lane_Enabled[6]{ false, false, false, false, false, false };
short CPS_Lane_Fault[6]{ 0, 0, 0, 0, 0, 0 };
short CPS_Lane_RD[6]{ 10000, 10000, 10000, 10000, 10000, 10000 };
bool CPS_Lane_Completed[6]{ false, false, false, false, false, false };
bool CPS_Lane_End_Signal[6]{ false, false, false, false, false, false };

//CPS Thread
bool isAvailable[6]{ true, true, true, true, true, true };
bool isAssigned[6]{ false, false, false, false, false, false };
bool isRunning[6]{ false, false, false, false, false, false };
int assignedLaneNumber[6]{ 1, 2, 3, 4, 5, 6 };
int assignedProcNumber[6]{ -1 , -1, -1, -1, -1, -1 };

bool sensor_connected[6]{ false, false, false, false, false, false };
bool sensor_fault[6]{ false, false, false, false, false, false };
std::queue<int> cps_queue = {}; //cps queue lane number.

bool activeLaneChanged[6] = { false, false, false, false, false, false };
std::string cpsSaveFolderName[6] = { "", "", "", "", "", ""};

std::mutex mutex_cps[6]{};
std::condition_variable cond_cps[6]{};
std::atomic<bool> cps_running[6]{ true, true, true, true, true, true };
bool cps_flag[6]{ false, false, false, false, false, false  };

std::string saveName;
std::string saveDirName;

std::string job_name;
std::string job_result_folder_name;
//images, pointcloud, and .log files
std::string job_result_log_file_name;

std::chrono::system_clock::time_point sensor_last_attempted_time = std::chrono::system_clock::now();
std::chrono::system_clock::time_point sensor_last_connected_time = std::chrono::system_clock::now();

//Debug
std::string DEBUG_IMG_PATH = "";
std::string DEBUG_PLY_PATH = "";
std::vector<std::string> DEBUG_IMG_FILES;
std::vector<std::string> DEBUG_PLY_FILES;
int DEBUG_CURRENT_INDEX = 0;
int DEBUG_MAX_INDEX = 0;

LONG WINAPI MyUnhandledExceptionFilter(EXCEPTION_POINTERS* ExceptionInfo) {
	HANDLE hFile = CreateFile(L"crash.dmp", GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (hFile != INVALID_HANDLE_VALUE) {
		MINIDUMP_EXCEPTION_INFORMATION mdei;
		mdei.ThreadId = GetCurrentThreadId();
		mdei.ExceptionPointers = ExceptionInfo;
		mdei.ClientPointers = FALSE;

		MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(),
			hFile, MiniDumpNormal, &mdei, nullptr, nullptr);

		CloseHandle(hFile);
	}

	return EXCEPTION_EXECUTE_HANDLER;
}

std::chrono::system_clock::time_point last_frame_get_time = std::chrono::system_clock::now();
bool blnGetNewFrame = false;
bool blnFrameFault = false;

std::atomic<bool> socket_running(true);

/*
 * Mutex and Conditional Variables
 *
 */

std::mutex mutex_logging;
std::condition_variable cond_logging;
//this one is for running/terminating thread.
std::atomic<bool> logging_running(true);

/*
 * Log Timer Variables
 */

std::queue<std::string> logQueue;
std::mutex queueMutex;
std::condition_variable cvLog;
std::atomic<bool> log_running(true);

std::mutex logMutex; // Mutex to protect log file writing
std::string currentLogFileName; // To track the current log file

std::vector<std::queue<std::string>> RDQueue = { std::queue<std::string>(), std::queue<std::string>(), std::queue<std::string>(), std::queue<std::string>(), std::queue<std::string>(), std::queue<std::string>() };
std::vector<std::mutex> RDQueueMutex(6);
std::vector<std::condition_variable> cvRDLog(6);
std::vector<bool> RDLog_running{ true, true, true, true, true, true };

std::vector<std::mutex> RDLogMutexes(6); // Mutex to protect log file writing
std::vector<std::string> currentRDLogFileName = {"","","","","",""}; // To track the current log file

std::string appID = "";
std::string appName = "CPS_TMini";

std::shared_ptr<VisionaryTMiniData> pDataHandler[6] = { std::make_shared<VisionaryTMiniData>(), std::make_shared<VisionaryTMiniData>(), std::make_shared<VisionaryTMiniData>(), std::make_shared<VisionaryTMiniData>(), std::make_shared<VisionaryTMiniData>(), std::make_shared<VisionaryTMiniData>() };
VisionaryDataStream dataStream[6] = {VisionaryDataStream(pDataHandler[0]), VisionaryDataStream(pDataHandler[1]), VisionaryDataStream(pDataHandler[2]), VisionaryDataStream(pDataHandler[3]), VisionaryDataStream(pDataHandler[4]), VisionaryDataStream(pDataHandler[5]) };
VisionaryControl visionaryControl[6] = {};

bool controlConnected[6] = { false, false, false, false, false, false };
bool dataConnected[6] = { false, false, false, false, false, false };

bool saveFlag = false;

class ThreadSafeQueue
{
private:
	std::queue <std::tuple<cv::Mat, pcl::PointCloud<pcl::PointXYZ>, bool, int, std::string, std::chrono::system_clock::time_point>> queue;
	std::mutex mtx;
	std::condition_variable cv;

public:
	void push(std::tuple<cv::Mat, pcl::PointCloud<pcl::PointXYZ>, bool, int, std::string, std::chrono::system_clock::time_point> value) {
		std::lock_guard<std::mutex> lock(mtx);
		queue.push(value);
		cv.notify_one(); // Notify the consumer
	}

	// Pop element from the queue (blocks if empty)
	std::tuple<cv::Mat, pcl::PointCloud<pcl::PointXYZ>, bool, int, std::string, std::chrono::system_clock::time_point> pop() {
		std::unique_lock<std::mutex> lock(mtx);
		cv.wait(lock, [this] { return !queue.empty(); }); // Wait until queue is non-empty
		std::tuple<cv::Mat, pcl::PointCloud<pcl::PointXYZ>, bool, int, std::string, std::chrono::system_clock::time_point> value = queue.front();
		queue.pop();
		return value;
	}

	int GetQueueLen()
	{
		std::lock_guard<std::mutex> lock(mtx);
		return queue.size();
	}

	void clear() {
		std::lock_guard<std::mutex> lock(mtx);
		while (!queue.empty()) {
			queue.pop();
		}
	}
};

ThreadSafeQueue tsq;

void logMessage(const std::string& message);

void my_handler(int s)
{

	logging_running.store(false);
	socket_running.store(false);

	cps_running[0].store(false);
	cps_running[1].store(false);
	cps_running[2].store(false);
	cps_running[4].store(false);
	cps_running[5].store(false);

	logMessage("Caught signal " + std::to_string(s));
	logMessage("Terminating tmini_process application triggered by Ctrl+C");

	log_running.store(false);

	exit(1);
}

std::vector<std::string> split(const std::string& str, char delimiter) {
	std::vector<std::string> tokens;
	std::stringstream ss(str);
	std::string token;

	while (std::getline(ss, token, delimiter)) {
		tokens.push_back(token);
	}
	return tokens;
}

auto print_time() -> std::string
{
	auto now = std::chrono::system_clock::now();

	// Convert to time_t to use with std::localtime
	std::time_t current_time = std::chrono::system_clock::to_time_t(now);

	// Convert to tm struct (local time)
	std::tm local_time = *std::localtime(&current_time);

	// Get the milliseconds part
	auto duration = now.time_since_epoch();
	auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() % 1000;

	std::ostringstream oss;
	oss << std::put_time(&local_time, "%Y%m%d_%H%M%S") << "_" << std::setw(3) << std::setfill('0') << milliseconds;

	// Get the formatted time as a string
	std::string formatted_time = oss.str();

	// Output formatted current time
	return formatted_time;
}

//Check if directory exists
auto dirExists(const char* const path) -> int
{
	struct stat info;

	int statRC = stat(path, &info);
	if (statRC != 0)
	{
		if (errno == ENOENT) { return 0; } // something along the path does not exist
		if (errno == ENOTDIR) { return 0; } // something in path prefix is not a dir
		return -1;
	}

	return (info.st_mode & S_IFDIR) ? 1 : 0;
}
//Check if directory exists, if not create one (returns true if created).
auto createDirectory_ifexists(std::string path) -> bool
{
	auto status = false;
	auto dirE = dirExists(path.c_str());
	if (dirE == 0)
	{
		auto ret = _mkdir(path.c_str());
		if (ret == -1) {
			logMessage("Error in creating directory!");
		}
		status = true;
	}
	return status;
}

// Get the current date as a string in the format YYYY-MM-DD
std::string getCurrentDate() {
	auto now = std::chrono::system_clock::now();
	std::time_t current_time = std::chrono::system_clock::to_time_t(now);
	std::tm local_time = *std::localtime(&current_time);
	std::ostringstream oss;
	oss << std::put_time(&local_time, "%Y%m%d");
	return oss.str();
}

// Function to write log entries to file
void logWriterThread() {
	std::ofstream logFile;

	createDirectory_ifexists(app_path + "/Log");

	while (log_running || !logQueue.empty()) {
		try
		{
			std::unique_lock<std::mutex> lock(queueMutex);
			cvLog.wait(lock, [] { return !logQueue.empty() || !log_running; });

			// Get the current date and check if it has changed
			std::string newLogFileName = app_path + "/Log/" + appName + "_log_" + getCurrentDate() + ".log";

			// If the date has changed, close the current log file and open a new one
			if (newLogFileName != currentLogFileName) {
				if (logFile.is_open()) {
					logFile.close(); // Close the previous file
				}
				logFile.open(newLogFileName, std::ios::out | std::ios::app); // Open new log file
				currentLogFileName = newLogFileName; // Update the current log file name
			}

			// Write all logs in the queue to the current file
			while (!logQueue.empty()) {
				logFile << logQueue.front() << std::endl;
				logQueue.pop();
			}
			lock.unlock();
		}
		catch (std::exception& ex)
		{
			std::cout << "Error in log thread : " << std::string(ex.what());
			//return false;
		}
		catch (...)
		{
			std::cout << "Unknown error in log thread";
			//return false;
		}
	}

	// Close the log file at the end
	if (logFile.is_open()) {
		logFile.close();
	}
}
// Function to log messages (called by other threads)
void logMessage(const std::string& message) {
	
	try
	{
		//std::cout << "Logging working " << message << "\n";
		std::stringstream ss;
		auto now = std::chrono::system_clock::now();
		auto time_since_epoch = now.time_since_epoch();
		auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_since_epoch).count() % 1000;

		std::time_t current_time = std::chrono::system_clock::to_time_t(now);
		std::tm local_time = *std::localtime(&current_time);

		ss << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S") << "." << std::setw(3) << std::setfill('0') << ms << " - " << message;

		{
			std::lock_guard<std::mutex> lock(queueMutex);
			logQueue.push(ss.str());
		}
		printf((ss.str() + "\n").c_str());
		cvLog.notify_one();
	}
	catch (std::exception& ex)
	{
		std::cout << "Error in log message : " << std::string(ex.what());
		//return false;
	}
}

//Lane별 Job Log
void RDWriterThread(int LaneNumber) {
	std::ofstream logFile;

	std::string logPath = app_path + "/Log";

	createDirectory_ifexists(logPath);
	
	//appName/Log/Lane_3
	logPath += "/Lane_" + std::to_string(LaneNumber);
	createDirectory_ifexists(logPath);

	while (RDLog_running[LaneNumber-1] || !RDQueue[LaneNumber - 1].empty()) 
	{
		try
		{
			std::unique_lock<std::mutex> lock(RDQueueMutex[LaneNumber - 1]);
			cvRDLog[LaneNumber-1].wait(lock, [&] { return !RDQueue[LaneNumber - 1].empty() || !RDLog_running[LaneNumber - 1]; });

			// Get the current date and check if it has changed
			std::string newLogFileName = logPath + "/" + appName + "_RD_Lane_" + std::to_string(LaneNumber) + "_" + getCurrentDate() + ".log";
			//std::string newLogFileName = app_path + "/Log/" + appName + "_log_" + getCurrentDate() + ".log";

			// If the date has changed, close the current log file and open a new one
			if (newLogFileName != currentRDLogFileName[LaneNumber - 1]) {
				if (logFile.is_open()) {
					logFile.close(); // Close the previous file
				}
				logFile.open(newLogFileName, std::ios::out | std::ios::app); // Open new log file
				currentRDLogFileName[LaneNumber - 1] = newLogFileName; // Update the current log file name
			}

			// Write all logs in the queue to the current file
			while (!RDQueue[LaneNumber - 1].empty()) {
				logFile << RDQueue[LaneNumber - 1].front() << std::endl;
				RDQueue[LaneNumber - 1].pop();
			}
			lock.unlock();
		}
		catch (std::exception& ex)
		{
			std::cout << "Error in RD " + std::to_string(LaneNumber) + " log thread : " << std::string(ex.what()) << "\n";
			//return false;
		}
		catch (...)
		{
			std::cout << "Unknown error in " + std::to_string(LaneNumber) + " log thread\n";
			//return false;
		}
	}

	// Close the log file at the end
	if (logFile.is_open()) {
		logFile.close();
	}
}
void RDMessage(const std::string& message, int LaneNumber, bool outMessage=false) {

	try
	{
		//std::cout << "Logging working " << message << "\n";
		std::stringstream ss;
		auto now = std::chrono::system_clock::now();
		auto time_since_epoch = now.time_since_epoch();
		auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_since_epoch).count() % 1000;

		std::time_t current_time = std::chrono::system_clock::to_time_t(now);
		std::tm local_time = *std::localtime(&current_time);

		ss << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S") << "." << std::setw(3) << std::setfill('0') << ms << " - " << message;

		{
			std::lock_guard<std::mutex> lock(RDQueueMutex[LaneNumber - 1]);
			RDQueue[LaneNumber - 1].push(ss.str());
		}
		if (outMessage) printf((ss.str() + "\n").c_str());
		cvRDLog[LaneNumber - 1].notify_one();
	}
	catch (std::exception& ex)
	{
		std::cout << "Error in RD " + std::to_string(LaneNumber) + " log message : " << std::string(ex.what());
		//return false;
	}
}



template <typename Duration>
auto print_time(tm t, Duration fraction) -> std::string
{
	using namespace std::chrono;
	char val[256];
	std::sprintf(val, "%04u%02u%02u_%02u%02u%02u_%03u", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec, static_cast<unsigned>(fraction / milliseconds(1)));
	std::string s(val);
	return s;
}
//Represent current datetime as filename
auto time_as_name() -> std::string
{
	//name is specific to be time. yyyyMMdd_hhMMss_msec
	std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
	std::chrono::system_clock::duration tp = now.time_since_epoch();
	tp -= std::chrono::duration_cast<std::chrono::seconds>(tp);
	time_t tt = std::chrono::system_clock::to_time_t(now);

	std::string t = print_time(*localtime(&tt), tp);
	return t;
}
auto time_as_name(std::chrono::system_clock::time_point ts) -> std::string
{
	//name is specific to be time. yyyyMMdd_hhMMss_msec
	//std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
	auto now = ts;
	std::chrono::system_clock::duration tp = now.time_since_epoch();
	tp -= std::chrono::duration_cast<std::chrono::seconds>(tp);
	time_t tt = std::chrono::system_clock::to_time_t(now);

	std::string t = print_time(*localtime(&tt), tp);
	return t;
}

auto time_up_to_millseconds(std::chrono::system_clock::time_point time_data) -> std::string {
	std::chrono::system_clock::duration tp = time_data.time_since_epoch();
	tp -= std::chrono::duration_cast<std::chrono::seconds>(tp);
	time_t tt = std::chrono::system_clock::to_time_t(time_data);

	std::string t = print_time(*localtime(&tt), tp);
	return t;
}

auto parseAppName(string path) -> bool
{
	try
	{
		mINI::INIFile inireader(path + "/INI/seoho.ini");
		mINI::INIStructure inidata;

		inireader.read(inidata);

		appID = inidata.get("ID").get("APP");
		return true;
	}
	catch (std::exception& ex)
	{
		std::cout << "Error in parsing INI file : " << std::string(ex.what());
		return false;
	}
}
auto parseINI(string path) -> bool
{
	try
	{
		mINI::INIFile inireader(path + "/INI/seoho.ini");
		mINI::INIStructure inidata;

		inireader.read(inidata);

		appID = inidata.get("ID").get("APP");
		//port
		//sensor_port = inidata.get("SENSOR").get("PORT");
		//logMessage("Port: " + sensor_port);

		//socket port
		SOCKET_PORT = std::stoi(inidata.get("SOCKET").get("SERVER_PORT"));
		logMessage("socket ip,port: " + SOCKET_IP + " , " + std::to_string(SOCKET_PORT));
		
		// ==== 로깅 정책 파라미터 ====

		FRAMES_PER_BUCKET = std::stoi(inidata.get("SYSTEM").get("FRAMES_PER_BUCKET"));     // 1m 구간당 저장 개수
		STABLE_DELTA_MM = std::stoi(inidata.get("SYSTEM").get("STABLE_DELTA_MM"));      // 정차 판정 이동량
		STABLE_FRAMES = std::stoi(inidata.get("SYSTEM").get("STABLE_FRAMES"));         // 정차 판정 프레임 수
		BUCKET_HYST_MM = std::stoi(inidata.get("SYSTEM").get("BUCKET_HYST_MM"));       // 버킷 전환 (mm)
		//bool ENABLE_SAVE_LOG = inidata.get("SYSTEM").get("ENABLE_SAVE_LOG_BIT");
		//bool DEFAULT_ENABLE = inidata.get("SYSTEM").get("DEFAULT_ENABLE");      // PLC 신호 없을 때 기본값

		TARGET_STOP_POSITION_MM = std::stoi(inidata.get("SYSTEM").get("TARGET_STOP_POSITION_MM"));

		ENABLE_SAVE_ALL = inidata.get("SYSTEM").get("ENABLE_SAVE_ALL") == "1" ? true : false;
		if (ENABLE_SAVE_ALL) logMessage("Enable Save All Data!");

		MODE_DEBUG = inidata.get("DEBUG").get("MODE_DEBUG") == "1" ? true : false;
		if (MODE_DEBUG) logMessage("Debug mode enabled!");

		DEBUG_SAMPLE_JOB = inidata.get("DEBUG").get("DEBUG_SAMPLE_JOB");

		DEBUG_CONVERT_PCL_RANGE = inidata.get("DEBUG").get("DEBUG_CONVERT_PCL_RANGE") == "1" ? true : false;

		DEBUG_SENSOR_POSITION = inidata.get("DEBUG").get("DEBUG_SENSOR_POSITION");
		
		DEBUG_WITH_FILES = inidata.get("DEBUG").get("DEBUG_WITH_FILES") == "1" ? true : false;

		DEBUG_PATH = inidata.get("DEBUG").get("DEBUG_PATH");

		DEBUG_BATCH_JOB = inidata.get("DEBUG").get("DEBUG_BATCH_JOB") == "1" ? true : false;
		if (DEBUG_BATCH_JOB) logMessage("Debug mode in batch mode!");
		DEBUG_BATCH_ROOT_DIR = inidata.get("DEBUG").get("DEBUG_BATCH_ROOT_DIR");
		DEBUG_BATCH_SAVE_DIR = inidata.get("DEBUG").get("DEBUG_BATCH_SAVE_DIR");

		if (MODE_DEBUG || DEBUG_WITH_FILES || DEBUG_BATCH_JOB) logMessage("Debug Sensor Pos: " + DEBUG_SENSOR_POSITION);

		logMessage("INI Configuration Complete!");
	}
	catch (std::exception& e)
	{
		logMessage("Error in parsing INI file : " + std::string(e.what()));
		return false;
	}
	return true;
}
void parseIPINI(string path)
{
	mINI::INIFile file(path + "/INI/IP_LIST.ini");
	mINI::INIStructure ini;
	if (file.read(ini))
	{
		try
		{
			
			for (int i = 0; i < 7; i++)
			{
				auto ip = ini.get("IP").get("IP" + std::to_string(i + 1));
				if (ip != "") {
					IP_ADDRESSES.push_back(ip);
				}
			}
			logMessage("IP INI Configuration Complete!");
		}
		catch (std::exception& ex)
		{
			logMessage("Error: " + std::string(ex.what()) + " from reading INI/IP_LIST.ini");
		}
		catch (...)
		{
			logMessage("Unknown error reading from INI/IP_LIST.ini");
		}
	}
	else
	{
		logMessage("Failed to read INI/IP_LIST.ini file");
	}
}

std::wstring s2ws(const std::string& s) //string -> LPCWSTR needs first conversion to wstring then to LPCWSTR.
{
	int len;
	int slength = (int)s.length() + 1;
	len = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, 0, 0);
	wchar_t* buf = new wchar_t[len];
	MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, buf, len);
	std::wstring r(buf);
	delete[] buf;
	return r;
}

template <size_t N1, size_t N2>
std::bitset <N1 + N2> concat(const std::bitset <N1>& b1, const std::bitset <N2>& b2) {
	std::string s1 = b1.to_string();
	std::string s2 = b2.to_string();
	return std::bitset <N1 + N2>(s1 + s2);
}

template<std::size_t B>
long bitset_to_long(const std::bitset<B>& b) {
	struct { long x : B; } s;
	return s.x = b.to_ulong();
}

void reset_jobVariables() { ; }
void reset_recvVariables() { ; }

std::vector<std::string> getAllFiles(boost::filesystem::path const& root, std::string const& ext)
{
	std::vector<boost::filesystem::path> paths;
	std::vector<std::string> full_paths;

	if (boost::filesystem::exists(root) && boost::filesystem::is_directory(root))
	{
		for (auto const& entry : boost::filesystem::recursive_directory_iterator(root))
		{
			if (boost::filesystem::is_regular_file(entry) && entry.path().extension() == ext)
			{
				paths.emplace_back(entry.path().filename());
				full_paths.push_back(entry.path().string());
			}
		}
	}
	return full_paths;
}
std::vector<std::string> ListSubDirectories(const std::string& directoryPath) {
	std::vector<std::string> subDirs;

	boost::filesystem::path dir(directoryPath);
	if (boost::filesystem::exists(dir) && boost::filesystem::is_directory(dir)) {
		// Iterate through the directory
		for (boost::filesystem::directory_entry& entry : boost::filesystem::directory_iterator(dir)) {
			if (boost::filesystem::is_directory(entry.status())) {
				subDirs.push_back(entry.path().string());
			}
		}
	}
	else {
		std::cerr << "Provided path is not a valid directory: " << directoryPath << std::endl;
	}

	return subDirs;
}
std::vector<std::string> getSubdirectoriesName(const std::string& path) {
	std::vector<std::string> subdirs;

	try {
		for (const auto& entry : fs::directory_iterator(path)) {
			if (entry.is_directory()) {
				subdirs.push_back(entry.path().filename().string());
			}
		}
	}
	catch (const fs::filesystem_error& e) {
		std::cerr << "Error: " << e.what() << std::endl;
	}

	return subdirs;
}
std::string makeJobFolderName(int laneNumber)
{
	std::string baseDir = "CPS";
	createDirectory_ifexists(baseDir);

	std::string job_info = baseDir + "/" + print_time() + std::string("_");
	job_info.append("CPS_");
	
	//TODO: Block Number

	job_info.append(std::to_string(laneNumber));
	createDirectory_ifexists(job_info);

	return job_info;
}

auto makeCommand(char* sendbuf) -> void {
	try
	{
		//send heartbeat
			//increment heartbeat
		heartbeat_ += 1;
		if (heartbeat_ > 65535) heartbeat_ = 0;
		//std::cout << "Sending heartbeat: " << std::to_string(heartbeat_) << std::endl;
		std::bitset<16>heartbeat_bit(heartbeat_);
		const std::string temp_str = heartbeat_bit.to_string();
		std::bitset<8>first_half(temp_str.substr(0, temp_str.size() / 2));
		std::bitset<8>second_half(temp_str.substr(temp_str.size() / 2, temp_str.size()));

		sendbuf[0] = static_cast<char>(first_half.to_ulong());
		sendbuf[1] = static_cast<char>(second_half.to_ulong());

		std::bitset<8>op_byte_2(0);
		if (CPS_Lane_Enabled[0]) op_byte_2.set(0);
		if (CPS_Lane_Enabled[1]) op_byte_2.set(1);
		if (CPS_Lane_Enabled[2]) op_byte_2.set(2);
		if (CPS_Lane_Enabled[3]) op_byte_2.set(3);
		if (CPS_Lane_Enabled[4]) op_byte_2.set(4);
		if (CPS_Lane_Enabled[5]) op_byte_2.set(5);
		
		sendbuf[2] = static_cast<char>(op_byte_2.to_ulong());

		{
			std::bitset<16>short_bit_arr(CPS_Lane_RD[0]);
			const std::string temp_str = short_bit_arr.to_string();
			std::bitset<8>first_half(temp_str.substr(0, temp_str.size() / 2));
			std::bitset<8>second_half(temp_str.substr(temp_str.size() / 2, temp_str.size()));
			sendbuf[3] = static_cast<char>(first_half.to_ulong());
			sendbuf[4] = static_cast<char>(second_half.to_ulong());
		}
		{
			std::bitset<16>short_bit_arr(CPS_Lane_RD[1]);
			const std::string temp_str = short_bit_arr.to_string();
			std::bitset<8>first_half(temp_str.substr(0, temp_str.size() / 2));
			std::bitset<8>second_half(temp_str.substr(temp_str.size() / 2, temp_str.size()));
			sendbuf[5] = static_cast<char>(first_half.to_ulong());
			sendbuf[6] = static_cast<char>(second_half.to_ulong());
		}
		{
			std::bitset<16>short_bit_arr(CPS_Lane_RD[2]);
			const std::string temp_str = short_bit_arr.to_string();
			std::bitset<8>first_half(temp_str.substr(0, temp_str.size() / 2));
			std::bitset<8>second_half(temp_str.substr(temp_str.size() / 2, temp_str.size()));
			sendbuf[7] = static_cast<char>(first_half.to_ulong());
			sendbuf[8] = static_cast<char>(second_half.to_ulong());
		}
		{
			std::bitset<16>short_bit_arr(CPS_Lane_RD[3]);
			const std::string temp_str = short_bit_arr.to_string();
			std::bitset<8>first_half(temp_str.substr(0, temp_str.size() / 2));
			std::bitset<8>second_half(temp_str.substr(temp_str.size() / 2, temp_str.size()));
			sendbuf[9] = static_cast<char>(first_half.to_ulong());
			sendbuf[10] = static_cast<char>(second_half.to_ulong());
		}
		{
			std::bitset<16>short_bit_arr(CPS_Lane_RD[4]);
			const std::string temp_str = short_bit_arr.to_string();
			std::bitset<8>first_half(temp_str.substr(0, temp_str.size() / 2));
			std::bitset<8>second_half(temp_str.substr(temp_str.size() / 2, temp_str.size()));
			sendbuf[11] = static_cast<char>(first_half.to_ulong());
			sendbuf[12] = static_cast<char>(second_half.to_ulong());
		}
		{
			std::bitset<16>short_bit_arr(CPS_Lane_RD[5]);
			const std::string temp_str = short_bit_arr.to_string();
			std::bitset<8>first_half(temp_str.substr(0, temp_str.size() / 2));
			std::bitset<8>second_half(temp_str.substr(temp_str.size() / 2, temp_str.size()));
			sendbuf[13] = static_cast<char>(first_half.to_ulong());
			sendbuf[14] = static_cast<char>(second_half.to_ulong());
		}

		std::bitset<8>op_byte_15(0);
		if (CPS_Lane_Completed[0]) op_byte_15.set(0);
		if (CPS_Lane_Completed[1]) op_byte_15.set(1);
		if (CPS_Lane_Completed[2]) op_byte_15.set(2);
		if (CPS_Lane_Completed[3]) op_byte_15.set(3);
		if (CPS_Lane_Completed[4]) op_byte_15.set(4);
		if (CPS_Lane_Completed[5]) op_byte_15.set(5);

		sendbuf[15] = static_cast<char>(op_byte_15.to_ulong());

		{
			std::bitset<16>short_bit_arr(CPS_Lane_Fault[0]);
			const std::string temp_str = short_bit_arr.to_string();
			std::bitset<8>first_half(temp_str.substr(0, temp_str.size() / 2));
			std::bitset<8>second_half(temp_str.substr(temp_str.size() / 2, temp_str.size()));
			sendbuf[21] = static_cast<char>(first_half.to_ulong());
			sendbuf[22] = static_cast<char>(second_half.to_ulong());
		}
		{
			std::bitset<16>short_bit_arr(CPS_Lane_Fault[1]);
			const std::string temp_str = short_bit_arr.to_string();
			std::bitset<8>first_half(temp_str.substr(0, temp_str.size() / 2));
			std::bitset<8>second_half(temp_str.substr(temp_str.size() / 2, temp_str.size()));
			sendbuf[23] = static_cast<char>(first_half.to_ulong());
			sendbuf[24] = static_cast<char>(second_half.to_ulong());
		}
		{
			std::bitset<16>short_bit_arr(CPS_Lane_Fault[2]);
			const std::string temp_str = short_bit_arr.to_string();
			std::bitset<8>first_half(temp_str.substr(0, temp_str.size() / 2));
			std::bitset<8>second_half(temp_str.substr(temp_str.size() / 2, temp_str.size()));
			sendbuf[25] = static_cast<char>(first_half.to_ulong());
			sendbuf[26] = static_cast<char>(second_half.to_ulong());
		}
		{
			std::bitset<16>short_bit_arr(CPS_Lane_Fault[3]);
			const std::string temp_str = short_bit_arr.to_string();
			std::bitset<8>first_half(temp_str.substr(0, temp_str.size() / 2));
			std::bitset<8>second_half(temp_str.substr(temp_str.size() / 2, temp_str.size()));
			sendbuf[27] = static_cast<char>(first_half.to_ulong());
			sendbuf[28] = static_cast<char>(second_half.to_ulong());
		}
		{
			std::bitset<16>short_bit_arr(CPS_Lane_Fault[4]);
			const std::string temp_str = short_bit_arr.to_string();
			std::bitset<8>first_half(temp_str.substr(0, temp_str.size() / 2));
			std::bitset<8>second_half(temp_str.substr(temp_str.size() / 2, temp_str.size()));
			sendbuf[29] = static_cast<char>(first_half.to_ulong());
			sendbuf[30] = static_cast<char>(second_half.to_ulong());
		}
		{
			std::bitset<16>short_bit_arr(CPS_Lane_Fault[5]);
			const std::string temp_str = short_bit_arr.to_string();
			std::bitset<8>first_half(temp_str.substr(0, temp_str.size() / 2));
			std::bitset<8>second_half(temp_str.substr(temp_str.size() / 2, temp_str.size()));
			sendbuf[31] = static_cast<char>(first_half.to_ulong());
			sendbuf[32] = static_cast<char>(second_half.to_ulong());
		}

	}
	catch (std::exception& ex)
	{
		logMessage("Error in making command : " + std::string(ex.what()));
	}
	catch (...)
	{
		logMessage("Unknown error in making command");
	}
}
void parseCommand(char recvbuf[])
{
	try
	{
		char* recvBuf = &recvbuf[0];

		//heartbeat section
		{
			std::bitset<8> byte0(recvBuf[0]);
			std::bitset<8> byte1(recvBuf[1]);
			std::bitset<16> temp_heartbeat_ = concat(byte0, byte1);
			heartbeat_ = temp_heartbeat_.to_ulong();
			//std::cout << "Received heartbeat: " << std::to_string(heartbeat_) << std::endl;
		}
		
		{
			std::bitset<8> byte2(recvBuf[2]);	
			for (int i = 0; i < 6; i++)
			{
				bool tempVal = (bool)byte2[i];
				if (tempVal != CPS_Lane_Enable[i] && tempVal)
				{
					logMessage("CPS Lane " + std::to_string(i + 1) + " Enable Changed from " + std::to_string(CPS_Lane_Enable[i]) + " to " + std::to_string(tempVal));
					//trigger CPS thread.

					std::unique_lock<std::mutex> lock(mutex_cps[i]);
					cps_flag[i] = true;
					cond_cps[i].notify_one();

					saveFlag = true;
					cond_logging.notify_one();

					CPS_Lane_RD[i] = 10000;
					cpsSaveFolderName[i] = makeJobFolderName(i + 1);

					logMessage("CPS on Lane " + std::to_string(i + 1) + " is assigned to CPS");
					logMessage("CPS on Lane " + std::to_string(i + 1) + " Folder: " + cpsSaveFolderName[i]);
				}
				else if (tempVal != CPS_Lane_Enable[i] && !tempVal) //falling edge.
				{
					cps_flag[i] = false;
					cpsSaveFolderName[i] = "";

					CPS_Lane_Completed[i] = false;
					//CPS_Lane_RD[i] = -10000;
					CPS_Lane_Fault[i] = -10000;

					logMessage("CPS Disabled on Lane " + std::to_string(i + 1));
				}
				CPS_Lane_Enable[i] = tempVal;
			}

			//let it sleep
			if (saveFlag)
			{
				if (!(CPS_Lane_Enable[0] || CPS_Lane_Enable[1] || CPS_Lane_Enable[2] || CPS_Lane_Enable[3] || CPS_Lane_Enable[4] || CPS_Lane_Enable[5]))
					saveFlag = false;
			}
		}
		//CPS End Signal
		{
			//byte 3
			std::bitset<8> byte3(recvBuf[3]);
			CPS_Lane_End_Signal[0] = (bool)byte3[0];
			CPS_Lane_End_Signal[1] = (bool)byte3[1];
			CPS_Lane_End_Signal[2] = (bool)byte3[2];
			CPS_Lane_End_Signal[3] = (bool)byte3[3];
			CPS_Lane_End_Signal[4] = (bool)byte3[4];
			CPS_Lane_End_Signal[5] = (bool)byte3[5];
		}

		{
			//active lane section
			{
				std::bitset<8> byte4(recvBuf[4]);
				std::bitset<8> byte5(recvBuf[5]);
				std::bitset<16> temp_active_lane_ = concat(byte4, byte5);
				if (temp_active_lane_.to_ulong() != Active_Job_Lane && temp_active_lane_.to_ulong() > 0)
				{
					logMessage("Active Job Lane Changed from " + std::to_string(Active_Job_Lane) + " to " + std::to_string(temp_active_lane_.to_ulong()));

					//reset here
					CPS_Lane_Completed[temp_active_lane_.to_ulong() - 1] = false;

					if (isRunning[0]) activeLaneChanged[0] = true;
					if (isRunning[1]) activeLaneChanged[1] = true;
					if (isRunning[2]) activeLaneChanged[2] = true;
					if (isRunning[3]) activeLaneChanged[3] = true;
					if (isRunning[4]) activeLaneChanged[4] = true;
					if (isRunning[5]) activeLaneChanged[5] = true;

				}
				Active_Job_Lane = temp_active_lane_.to_ulong();
			}
		}

		return;
	}
	catch (std::exception& ex)
	{
		logMessage("Error: " + std::string(ex.what()) + " from parsing command.");
		return;
	}
	catch (...)
	{
		logMessage("Unknown error from parsing command.");
		return;
	}
}

struct cloud_data
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr = NULL;
	int pointSize = 0;

	std::string chassisType = "";

	pcl::PointXYZ center = pcl::PointXYZ(0, 0, 0);

	int max_x = 0, min_x = 0;
	int max_y = 0, min_y = 0;
	int max_z = 0, min_z = 0;

	int len_x = 0, len_y = 0, len_z = 0;
	int len_x_min_z = 0;

	int max_x_at_max_y = 0;
	int min_x_at_max_y = 0;

	pcl::PointXYZ max_x_at_min_z = pcl::PointXYZ(0, 0, 0);
	pcl::PointXYZ min_x_at_min_z = pcl::PointXYZ(0, 0, 0);

	int midBreakPoint = 0;

	bool valid = false;
};

bool pc_passThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr pcInput,
	float minV, float maxV, std::string axis, pcl::PointCloud<pcl::PointXYZ>::Ptr pcResult) {

	pcl::PassThrough<pcl::PointXYZ> pass;
	auto proc = false;
	auto minVal = min(minV, maxV);
	auto maxVal = max(minV, maxV);

	if (abs(maxVal - minVal) > DBL_EPSILON)
	{
		pass.setInputCloud(pcInput);
		pass.setFilterFieldName(axis);
		pass.setFilterLimits(minVal, maxVal); //min, max
		pass.filter(*pcResult);
		proc = true;
	}

	return proc;
}

bool pc_passThrough(
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcInput,
	float minX, float maxX,
	float minY, float maxY,
	float minZ, float maxZ,
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcResult)
{
	pcl::PassThrough<pcl::PointXYZ> pass;
	pcl::PointCloud<pcl::PointXYZ>::Ptr retX(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr retY(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr retZ(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr stepInput(new pcl::PointCloud<pcl::PointXYZ>);

	double minVal = 0.0;
	double maxVal = 0.0;
	bool proc = false;

	stepInput = pcInput;
	minVal = min(minX, maxX);
	maxVal = max(minX, maxX);
	if (abs(maxVal - minVal) > DBL_EPSILON)
	{
		pass.setInputCloud(stepInput);
		pass.setFilterFieldName("x");
		pass.setFilterLimits(minVal, maxVal); //min, max
		pass.filter(*retX);
		proc = true;
	}
	stepInput = proc ? retX : stepInput; proc = false;

	minVal = min(minY, maxY);
	maxVal = max(minY, maxY);
	if (abs(maxVal - minVal) > DBL_EPSILON)
	{
		pass.setInputCloud(stepInput);
		pass.setFilterFieldName("y");
		pass.setFilterLimits(minVal, maxVal); //min, max
		pass.filter(*retY);
		proc = true;
	}
	stepInput = proc ? retY : stepInput; proc = false;


	minVal = min(minZ, maxZ);
	maxVal = max(minZ, maxZ);
	if (abs(maxVal - minVal) > DBL_EPSILON)
	{
		pass.setInputCloud(stepInput);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(minVal, maxVal); //min, max
		pass.filter(*retZ);
		proc = true;
	}
	stepInput = proc ? retZ : stepInput; proc = false;

	//마지막은 없으면 없는데로 넣어줘야 한다.
	*pcResult = *stepInput;//

	return true;
}

bool pc_VoxelDown(
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcInput,
	float leafX, float leafY, float leafZ,
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcResult)
{
	if (pcInput->empty())
		return false;

	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(pcInput);
	sor.setLeafSize(leafX, leafY, leafZ);// 0.01 : 1cm
	sor.filter(*pcResult);

	return true;
}

bool pc_NoiseFilter(
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcInput,
	int MeanK_neighberCount,
	float StddevMulThresh,
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcResult)
{
	if (pcInput->empty())
		return false;

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(pcInput);
	sor.setMeanK(MeanK_neighberCount); //이웃한 점의 수
	sor.setStddevMulThresh(StddevMulThresh); //노이즈 표준편차 임계값
	sor.filter(*pcResult);

	//sor.setNegative(true);
	//sor.filter(*pcOutlier);

	return true;
}

bool pc_CountComp(const pcl::PointCloud<pcl::PointXYZ>::Ptr lhs, const pcl::PointCloud<pcl::PointXYZ>::Ptr rhs) {

	return lhs->points.size() < rhs->points.size();
}

bool pc_Clustering(
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcInput,
	int sizeMin, int sizeMax, float fTolerance,
	std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr>* listOut)
{
	if (pcInput->size() <= 0)
		return false;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_filtered(new pcl::PointCloud < pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_out(new pcl::PointCloud < pcl::PointXYZ>());

	cloud_filtered->resize(pcInput->size());

	pcl::copyPointCloud(*pcInput, *cloud_filtered);


	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setInputCloud(cloud_filtered);
	ec.setClusterTolerance(fTolerance); //0.02 - 2cm
	ec.setMinClusterSize(sizeMin); // 100 - 100
	ec.setMaxClusterSize(sizeMax); // 25000 - 25000
	ec.setSearchMethod(tree);
	ec.extract(cluster_indices);

	int sum = 0;
	int k = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{
			int idx = *pit;
			cloud_cluster->points.push_back(cloud_filtered->points[idx]); //*
		}
		//OutputDebugString("\n");
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;


		listOut->push_back(cloud_cluster);

		//sum += cloud_cluster->points.size();
	}
	listOut->sort(pc_CountComp);
	return true;
}

struct greater_pointXYZ_x
{
	inline bool operator() (const pcl::PointXYZ left, const pcl::PointXYZ right)
	{
		return (left.x > right.x);
	}
};
struct greater_pointXYZ_z
{
	inline bool operator() (const pcl::PointXYZ left, const pcl::PointXYZ right)
	{
		return (left.z > right.z);
	}
};

auto get_center_of_cloud(const pcl::PointCloud<pcl::PointXYZ> input, int& max_x, int& min_x, int& max_y, int& min_y, int& max_z, int& min_z) -> pcl::PointXYZ
{
	//Get max min for each axis
	auto loc_max_x = -100000;
	auto loc_min_x = 100000;

	auto loc_max_y = -100000;
	auto loc_min_y = 100000;

	auto loc_max_z = -100000;
	auto loc_min_z = 100000;

	for (auto it = input.points.begin(); it < input.points.end(); ++it)
	{
		auto val_x = (*it).x;
		auto val_y = (*it).y;
		auto val_z = (*it).z;

		if (val_x > loc_max_x)
			loc_max_x = val_x;
		if (val_x < loc_min_x)
			loc_min_x = val_x;

		if (val_y > loc_max_y)
			loc_max_y = val_y;
		if (val_y < loc_min_y)
			loc_min_y = val_y;

		if (val_z > loc_max_z)
			loc_max_z = val_z;
		if (val_z < loc_min_z)
			loc_min_z = val_z;
	}
	max_x = loc_max_x;
	min_x = loc_min_x;
	max_y = loc_max_y;
	min_y = loc_min_y;
	max_z = loc_max_z;
	min_z = loc_min_z;
	return pcl::PointXYZ((max_x - min_x) / 2 + min_x, (max_y - min_y) / 2 + min_y, (max_z - min_z) / 2 + min_z);
}

auto average_x(pcl::PointCloud<pcl::PointXYZ>::Ptr input) -> float
{
	float sum = 0;
	int count = 0;
	for (auto it = input->points.begin(); it < input->points.end(); ++it)
	{
		auto val = (*it).x;
		sum += val;
	}
	return sum / input->points.size();
}
auto get_max_min_x(pcl::PointCloud<pcl::PointXYZ>::Ptr input, int& maxX, int& minX) -> void
{
	int local_maxX = -10000;
	int local_minX = 10000;
	for (auto it = input->points.begin(); it < input->points.end(); ++it)
	{
		auto val = (*it).x;
		if (val > local_maxX) local_maxX = val;
		if (val < local_minX) local_minX = val;
	}

	maxX = local_maxX;
	minX = local_minX;
}
auto get_valid_max_x(pcl::PointCloud<pcl::PointXYZ>::Ptr input) -> int
{
	int local_maxX = -10000;
	int local_minX = 10000;

	for (auto it = input->points.begin(); it < input->points.end(); ++it)
	{
		auto val = (*it).x;
		if (val > local_maxX && val < 10000) local_maxX = val;
		if (val < local_minX && val > -10000) local_minX = val;
	}

	//slice through x to find first valid slice for min_z.
	for (int cx = local_maxX; cx > local_minX ; cx -= 10)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr slice_(new pcl::PointCloud<pcl::PointXYZ>);
		auto val = pc_passThrough(input, cx - 40, cx, "x", slice_);
		//if (save_path != std::string("")) pcl::io::savePCDFileBinaryCompressed(save_path + "_cone_slice_y_i_" + std::to_string(i) + "pc_" + std::to_string(slice_->points.size()) + ".pcd", *slice_);
		if (val)
		{
			if (slice_->points.size() > 30)
			{
				//get average val.
				auto avg_max_x = average_x(slice_);
				return (int)avg_max_x;

			}
		}
	}

	return local_maxX;
}
auto get_valid_min_x(pcl::PointCloud<pcl::PointXYZ>::Ptr input) -> int
{
	int local_maxX = -10000;
	int local_minX = 10000;

	for (auto it = input->points.begin(); it < input->points.end(); ++it)
	{
		auto val = (*it).x;
		if (val > local_maxX && val < 10000) local_maxX = val;
		if (val < local_minX && val > -10000) local_minX = val;
	}

	//slice through x to find first valid slice for min_z.
	for (int cx = local_minX; cx < local_maxX; cx += 10)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr slice_(new pcl::PointCloud<pcl::PointXYZ>);
		auto val = pc_passThrough(input, cx, cx + 40, "x", slice_);
		//if (save_path != std::string("")) pcl::io::savePCDFileBinaryCompressed(save_path + "_cone_slice_y_i_" + std::to_string(i) + "pc_" + std::to_string(slice_->points.size()) + ".pcd", *slice_);
		if (val)
		{
			if (slice_->points.size() > 30)
			{
				//get average val.
				auto avg_min_x = average_x(slice_);
				return (int)avg_min_x;

			}
		}
	}

	return local_minX;
}

auto get_max_min_y(pcl::PointCloud<pcl::PointXYZ>::Ptr input, int& maxY, int& minY) -> void
{
	int local_maxY = -10000;
	int local_minY = 10000;
	for (auto it = input->points.begin(); it < input->points.end(); ++it)
	{
		auto val = (*it).y;
		if (val > local_maxY) local_maxY = val;
		if (val < local_minY) local_minY = val;
	}

	maxY = local_maxY;
	minY = local_minY;
}

auto average_z(pcl::PointCloud<pcl::PointXYZ>::Ptr input) -> float
{
	float sum = 0;
	int count = 0;
	for (auto it = input->points.begin(); it < input->points.end(); ++it)
	{
		auto val = (*it).z;
		sum += val;
	}
	return sum / input->points.size();
}
auto get_valid_max_z(pcl::PointCloud<pcl::PointXYZ>::Ptr input) -> int
{
	int local_maxZ = -10000;
	int local_minZ = 10000;

	for (auto it = input->points.begin(); it < input->points.end(); ++it)
	{
		auto val = (*it).z;
		if (val > local_maxZ && val < 10000) local_maxZ = val;
		if (val < local_minZ && val > -10000) local_minZ = val;
	}

	//slice through x to find first valid slice for min_z.
	for (int cz = local_maxZ; cz > local_minZ; cz -= 10)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr slice_(new pcl::PointCloud<pcl::PointXYZ>);
		auto val = pc_passThrough(input, cz - 40, cz, "z", slice_);
		//if (save_path != std::string("")) pcl::io::savePCDFileBinaryCompressed(save_path + "_cone_slice_y_i_" + std::to_string(i) + "pc_" + std::to_string(slice_->points.size()) + ".pcd", *slice_);
		if (val)
		{
			if (slice_->points.size() > 30)
			{
				//get average val.
				auto avg_max_z = average_z(slice_);
				return (int)avg_max_z;

			}
		}
	}

	return local_maxZ;
}
auto get_valid_min_z(pcl::PointCloud<pcl::PointXYZ>::Ptr input) -> int
{
	int local_maxZ = -10000;
	int local_minZ = 10000;

	for (auto it = input->points.begin(); it < input->points.end(); ++it)
	{
		auto val = (*it).z;
		if (val > local_maxZ && val < 10000) local_maxZ = val;
		if (val < local_minZ && val > -10000) local_minZ = val;
	}

	//slice through x to find first valid slice for min_z.
	for (int cz = local_minZ; cz < local_maxZ; cz += 10)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr slice_(new pcl::PointCloud<pcl::PointXYZ>);
		auto val = pc_passThrough(input, cz, cz + 40, "z", slice_);
		//if (save_path != std::string("")) pcl::io::savePCDFileBinaryCompressed(save_path + "_cone_slice_y_i_" + std::to_string(i) + "pc_" + std::to_string(slice_->points.size()) + ".pcd", *slice_);
		if (val)
		{
			if (slice_->points.size() > 30)
			{
				//get average val.
				auto avg_min_z = average_z(slice_);
				return (int)avg_min_z;

			}
		}
	}

	//at this point, just return the local min z.
	return local_minZ;


}
auto get_max_min_z(pcl::PointCloud<pcl::PointXYZ>::Ptr input, int& maxZ, int& minZ) -> void
{
	int local_maxZ = -10000;
	int local_minZ = 10000;
	for (auto it = input->points.begin(); it < input->points.end(); ++it)
	{
		auto val = (*it).z;
		if (val > local_maxZ) local_maxZ = val;
		if (val < local_minZ) local_minZ = val;
	}

	maxZ = local_maxZ;
	minZ = local_minZ;
}

auto get_closest_xz(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointXYZ refPT) -> pcl::PointXYZ
{
	auto min = 10000;
	pcl::PointXYZ closest(0.0f, 0.0f, 0.0f);
	for (auto it = input->points.begin(); it < input->points.end(); ++it)
	{
		auto val = sqrt(pow((*it).x - refPT.x, 2) + pow((*it).z - refPT.z, 2));
		if (val < min)
		{
			min = val;
			closest = (*it);
		}
	}
	return closest;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr makePCL_PointCloud(std::vector<visionary::PointXYZ> input)
{
	try
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud <pcl::PointXYZ>);

		pointCloud->points.resize(input.size());

		std::transform(input.begin(), input.end(), pointCloud->points.begin(),
			[](visionary::PointXYZ val) { return pcl::PointXYZ(val.x * 1000, val.y * 1000, val.z * 1000); });

		return pointCloud;
	}
	catch (std::exception& ex)
	{
		logMessage("std exception occurred in pcl pointer making from T-Mini data: " + std::string(ex.what()));
		return NULL;
	}
}

//for early logging files 
pcl::PointCloud<pcl::PointXYZ>::Ptr makePCL_PointCloud(pcl::PointCloud<pcl::PointXYZ> input, bool convert = false)
{
	//auto dt_now = std::chrono::system_clock::now();
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ >);

	pointCloud->points.resize(input.size());

	if (convert)
	{
		std::transform(input.points.begin(), input.points.end(), pointCloud->points.begin(),
			[](pcl::PointXYZ val) { return pcl::PointXYZ(val.x * 1000, val.y * 1000, val.z * 1000); });
	}
	else
	{
		std::transform(input.points.begin(), input.points.end(), pointCloud->points.begin(),
			[](pcl::PointXYZ val) { return pcl::PointXYZ(val.x, val.y, val.z); });
	}
	//auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - dt_now).count();
	//std::cout << "make dur ms: " << std::to_string(dur) << std::endl;

	//pcl::io::savePCDFileBinaryCompressed("temp.pcd", *pointCloud);
	return pointCloud;
}

bool save_to_drive(bool isLeftSensor, int procIndex, cv::Mat oImage, pcl::PointCloud<pcl::PointXYZ> pointCloud, std::string msg, std::chrono::system_clock::time_point frameTime)
{
	try 
	{
		auto baseFolder = cpsSaveFolderName[procIndex];

		auto laneNum = procIndex + 1;

		baseFolder += "/Lane" + std::to_string(laneNum);
		createDirectory_ifexists(baseFolder);

		if (isLeftSensor)
		{
			baseFolder += "/LeftSensor";
		}
		else
		{
			baseFolder += "/RightSensor";
		}

		createDirectory_ifexists(baseFolder);

		auto SaveImageDir = baseFolder + "/Image";
		auto SaveDepthDir = baseFolder + "/Depth";

		createDirectory_ifexists(SaveImageDir);
		createDirectory_ifexists(SaveDepthDir);

		auto timeNow = time_as_name(frameTime);

		std::string pos = "RL";
		if (!isLeftSensor) pos = "RR";

		if (!oImage.empty()) cv::imwrite(SaveImageDir + "/" + timeNow + "_CPS_" + pos + + "_LANE" + std::to_string(laneNum) + "_Image.jpg", oImage);
		std::string plyFilePath = SaveDepthDir + "/" + timeNow + "_CPS_" + pos + + "_LANE" + std::to_string(laneNum) + "_Depth.pcd";
		if (pointCloud.points.size() > 0) pcl::io::savePCDFileBinaryCompressed(plyFilePath.c_str(), pointCloud);
		
		return true;
	}
	catch (...)
	{
		logMessage("[save_to_drive] Exception occurred.");
		return false;
	}
}

void data_save_thread()
{
	logMessage("Data Logging Thread Activated");
	try
	{
		while (logging_running.load())
		{
			//Safe blocking op.
			std::unique_lock<std::mutex> lock(mutex_logging);
			bool res = cond_logging.wait_for(lock,
				std::chrono::seconds(3600),
				[]() { return saveFlag; });

			//purposely woken up.
			int proc_count = 5;
			if (res)
			{
				while (saveFlag)
				{
					try
					{
						if (tsq.GetQueueLen() > 0)
						{
							//in blocked state until item is placed in queue.
							auto dataTup = tsq.pop();

							cv::Mat oImage; cv::Mat resImage; pcl::PointCloud<pcl::PointXYZ> pointCloud;
							bool isLeftSensor = false; std::string msg;
							int procIndex = -1;

							oImage = std::get<0>(dataTup);
							pointCloud = std::get<1>(dataTup);
							isLeftSensor = std::get<2>(dataTup);
							procIndex = std::get<3>(dataTup);
							msg = std::get<4>(dataTup);
							auto frameTime = std::get<5>(dataTup);

							bool save_res = save_to_drive(isLeftSensor, procIndex, oImage, pointCloud, msg, frameTime);
							if (!save_res) logMessage("Failed to save data to drive!");
							else logMessage("[Data-Save-Thread] Data saved successfully for " + msg + " at " + time_as_name(frameTime) + ", Lane-" + std::to_string(procIndex + 1));
							//do not hug the process.
							std::this_thread::sleep_for(std::chrono::milliseconds(25));
						}
					}
					catch (std::exception& ex)
					{
						logMessage("[Data-Save-Thread] " + std::string(ex.what()));
						std::this_thread::sleep_for(std::chrono::milliseconds(10));
					}
					catch (...)
					{
						logMessage("[Data-Save-Thread] Unknown Exception!");
						std::this_thread::sleep_for(std::chrono::milliseconds(10));
					}
				}
			}
		}

		logMessage("Data Logging Thread Deactivated");
	}
	catch (std::exception& ex)
	{
		logMessage("[Data-Save-Thread] " + std::string(ex.what()));
	}
	catch (...)
	{
		logMessage("[Data-Save-Thread] Unknown Exception!");
	}	
}

void clear_proc_status(int procNum)
{
	int i = procNum - 1;
	cps_flag[i] = false;

	isRunning[i] = false;
	activeLaneChanged[i] = false;

	cpsSaveFolderName[i] = "";
	CPS_Lane_Enabled[i] = false;
	
	logMessage("CPS - Proc: " + std::to_string(procNum) + " Status Cleared");
}

using Clock = std::chrono::steady_clock;

struct CPSFrameResult { //샤시 2대 일 떄 동시 판단을 위해서 2개까지 ..
	std::vector<int>         clusterCounts;   // up to 2 clusters (point counts) 
	std::vector<cv::Point3f> tailPoints;      // up to 2 tailpoints (nearest first)
	long long                processing_ms = 0;
};

static constexpr float VOX_MM = 5.0f;
static constexpr int   SOR_K = 30;
static constexpr float SOR_STD = 1.0f;        // previously 1.5

static constexpr float CLUSTER_TOL = 120.0f;  // previously 60
static constexpr int   MIN_PTS = 30;      // previously 50

// ==================== Filtering / Denoising ====================
pcl::PointCloud<pcl::PointXYZ>::Ptr removeNoise(
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud,
	float x_min, float x_max,
	float y_min, float y_max,
	float z_min, float z_max,
	const std::string& debug_filename)
{
	if (!inputCloud || inputCloud->empty())
		return std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

	auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
		new pcl::PointCloud<pcl::PointXYZ>(*inputCloud));

	pcl::PassThrough<pcl::PointXYZ> pass;

	pass.setInputCloud(cloud); pass.setFilterFieldName("x");
	pass.setFilterLimits(x_min, x_max); pass.filter(*cloud);

	pass.setInputCloud(cloud); pass.setFilterFieldName("y");
	pass.setFilterLimits(y_min, y_max); pass.filter(*cloud);

	pass.setInputCloud(cloud); pass.setFilterFieldName("z");
	pass.setFilterLimits(z_min, z_max); pass.filter(*cloud);

	// Voxel downsample
	{
		pcl::VoxelGrid<pcl::PointXYZ> vox; vox.setInputCloud(cloud);
		vox.setLeafSize(VOX_MM, VOX_MM, VOX_MM); vox.filter(*cloud);
	}
	// Statistical outlier removal
	{
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(cloud);
		sor.setMeanK(SOR_K);
		sor.setStddevMulThresh(SOR_STD);
		sor.filter(*cloud);
	}

	if (!debug_filename.empty())
		pcl::io::savePCDFileBinaryCompressed(debug_filename, *cloud);

	return cloud;
}

// ==================== Tailpoint from clusters ====================
static cv::Point3f tailFromCluster_ZPercentile_YMeanNearest(
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster,
	float percentile = 0.05f,
	const std::string& debug_slice_filename = "")
{
	if (!cluster || cluster->empty()) return { -9999, -9999, -9999 };

	std::vector<float> zs; zs.reserve(cluster->size());
	for (const auto& p : cluster->points) zs.push_back(p.z);

	size_t k = (size_t)std::clamp((int)std::floor(zs.size() * percentile), 0, (int)zs.size() - 1);
	std::nth_element(zs.begin(), zs.begin() + k, zs.end());
	float z_thresh = zs[k];

	auto sliced = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	sliced->reserve(cluster->size());
	for (const auto& p : cluster->points) if (p.z <= z_thresh) sliced->push_back(p);

	if (sliced->empty()) return { -9999, -9999, -9999 };
	if (!debug_slice_filename.empty())
		pcl::io::savePCDFileBinaryCompressed(debug_slice_filename, *sliced);

	float my = 0.f; for (const auto& p : sliced->points) my += p.y; my /= sliced->size();

	float best = 1e9f; pcl::PointXYZ bp;
	for (const auto& p : sliced->points) {
		float d = std::fabs(p.y - my);
		if (d < best) { best = d; bp = p; }
	}
	return { bp.x, bp.y, bp.z };
}

std::vector<cv::Point3f> detectChassisTailCenters(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	float cluster_tolerance_mm,
	int min_points,
	float z_slice_percent,
	const std::string& debug_base,
	std::vector<int>* cluster_counts)
{
	std::vector<cv::Point3f> results;
	if (cluster_counts) cluster_counts->clear();
	if (!cloud || cloud->empty()) return results;

	// Euclidean clustering
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> clusters_idx;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(cluster_tolerance_mm);
	ec.setMinClusterSize(min_points);
	ec.setMaxClusterSize(250000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(clusters_idx);

	if (clusters_idx.empty()) return results;

	struct CI { int n; pcl::PointIndices idx; };
	std::vector<CI> cs; cs.reserve(clusters_idx.size());
	for (auto& id : clusters_idx) cs.push_back({ (int)id.indices.size(), id });
	std::sort(cs.begin(), cs.end(), [](const CI& a, const CI& b) { return a.n > b.n; });
	if (cs.size() > 2) cs.resize(2);

	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
	clusters.reserve(cs.size());
	for (size_t i = 0; i < cs.size(); ++i) {
		auto cl = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
		cl->reserve(cs[i].n);
		for (int id : cs[i].idx.indices) cl->push_back(cloud->points[(size_t)id]);
		if (cluster_counts) cluster_counts->push_back((int)cl->size());
		clusters.push_back(cl);
		if (!debug_base.empty())
			pcl::io::savePCDFileBinaryCompressed(debug_base + "_cluster" + std::to_string(i) + ".pcd", *cl);
	}

	for (size_t i = 0; i < clusters.size(); ++i) {
		auto tail = tailFromCluster_ZPercentile_YMeanNearest(
			clusters[i], std::clamp(z_slice_percent, 0.01f, 0.20f),
			debug_base.empty() ? "" : (debug_base + "_cluster" + std::to_string(i) + "_zsliced.pcd"));
		if (tail.z > -9000.f) results.push_back(tail);
	}

	// Nearer first
	std::sort(results.begin(), results.end(), [](const auto& a, const auto& b) { return a.z < b.z; });
	if (results.size() > 2) results.resize(2);
	return results;
}

CPSFrameResult processCPSFrame(
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& rawCloud,
	const bool LeftSide,
	const std::string& baseDir,
	const std::string& frameName,
	bool save)
{
	auto t0 = Clock::now();
	CPSFrameResult result;
	result.clusterCounts = { 0,0 };

	// ---- Tunables (relaxed for robustness) ----
	float ROI_X_Min = -2000.f, ROI_X_Max = 2000.f;
	float ROI_Y_Min = -1500.f, ROI_Y_Max = -1000.f;
	float ROI_Z_Min = 1500.f, ROI_Z_Max = 5000.f;

	if (LeftSide) { ROI_X_Max = -800.f; }
	else { ROI_X_Min = 800.f; }

	if (!rawCloud || rawCloud->empty()) {
		result.processing_ms = 0;
		return result;
	}

	// --- 0) Auto unit scaling
	pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>());
	src->reserve(rawCloud->size());
	float zmin = 1e9f, zmax = -1e9f;
	for (const auto& p : rawCloud->points) { zmin = std::min(zmin, p.z); zmax = std::max(zmax, p.z); }
	const float scale = (zmax < 50.f ? 1000.f : 1.f); // threshold 50 is safe (in mm it's always >>50)
	for (const auto& p : rawCloud->points) src->push_back(pcl::PointXYZ(p.x * scale, p.y * scale, p.z * scale));

	// --- 1) ROI + denoise
	std::string debugFiltered = (save && !baseDir.empty() && !frameName.empty())
		? (baseDir + "/" + frameName + "_filtered.pcd") : "";
	auto filtered = removeNoise(src, ROI_X_Min, ROI_X_Max, ROI_Y_Min, ROI_Y_Max, ROI_Z_Min, ROI_Z_Max, debugFiltered);

	// --- 2) Cluster & tail detection
	std::string debugBase = (save && !baseDir.empty() && !frameName.empty())
		? (baseDir + "/" + frameName) : "";
	auto tails = detectChassisTailCenters(filtered, CLUSTER_TOL, MIN_PTS, 0.05f, debugBase, &result.clusterCounts);

	// --- 3) Fallback...  if detection fails, return at least min-Z point from filtered cloud (avoid Z=-1)
	if (tails.empty() && filtered && !filtered->empty()) {
		float bestZ = 1e9f; pcl::PointXYZ bestP;
		for (const auto& q : filtered->points) { if (q.z < bestZ) { bestZ = q.z; bestP = q; } }
		tails.push_back(cv::Point3f(bestP.x, bestP.y, bestP.z));
	}

	// Nearer first, cap to 2
	std::sort(tails.begin(), tails.end(), [](const auto& a, const auto& b) { return a.z < b.z; });
	if (tails.size() > 2) tails.resize(2);
	result.tailPoints = tails;

	result.processing_ms = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t0).count();

	return result;
}

struct DistScore2 {
	float score = 0.f;       // higher = better distribution
	float coverage = 0.f;    // occ_total / total_cells
	float inner_ratio = 0.f; // occ_inner / occ_total
	int occ_total = 0;
	int occ_inner = 0;
	std::size_t n = 0;
};

static inline int popcount_u64(uint64_t x) {
#if defined(_MSC_VER)
	return (int)__popcnt64(x);
#elif defined(__GNUC__) || defined(__clang__)
	return __builtin_popcountll(x);
#else
	int c = 0; while (x) { x &= (x - 1); ++c; } return c;
#endif
}

inline DistScore2 distributeness_innerfill_fast(
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
	int nx = 32,
	int ny = 32,
	int border = 1,              // 1-cell border ring (try 1 or 2)
	std::size_t n_min = 50
) {
	DistScore2 out;
	if (!cloud) return out;
	out.n = cloud->size();
	if (out.n < n_min) return out;
	if (nx < 4 || ny < 4) return out;
	border = std::max(1, std::min(border, std::min(nx, ny) / 4));

	// Pass 1: bbox
	float xmin = std::numeric_limits<float>::infinity();
	float xmax = -xmin;
	float ymin = xmin;
	float ymax = -xmin;
	for (const auto& p : cloud->points) {
		xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x);
		ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
	}
	const float dx = xmax - xmin;
	const float dy = ymax - ymin;
	if (!(dx > 1e-9f) || !(dy > 1e-9f)) return out;

	const float sx = nx / dx;
	const float sy = ny / dy;

	const int cells = nx * ny;
	const int words = (cells + 63) / 64;

	thread_local std::vector<uint64_t> bits;
	if ((int)bits.size() != words) bits.assign(words, 0ULL);
	else std::fill(bits.begin(), bits.end(), 0ULL);

	// Pass 2: set occupancy bit
	for (const auto& p : cloud->points) {
		int ix = (int)((p.x - xmin) * sx);
		int iy = (int)((p.y - ymin) * sy);
		if (ix < 0) ix = 0; else if (ix >= nx) ix = nx - 1;
		if (iy < 0) iy = 0; else if (iy >= ny) iy = ny - 1;

		const int cell = iy * nx + ix;
		bits[cell >> 6] |= (1ULL << (cell & 63));
	}

	// Count total occupied cells (popcount)
	int occ_total = 0;
	for (uint64_t w : bits) occ_total += popcount_u64(w);
	out.occ_total = occ_total;
	if (occ_total == 0) return out;

	// Count occupied inner cells (exclude border ring) — fast scan by cell
	int occ_inner = 0;
	const int ix0 = border, ix1 = nx - border;
	const int iy0 = border, iy1 = ny - border;

	for (int iy = iy0; iy < iy1; ++iy) {
		int base = iy * nx;
		for (int ix = ix0; ix < ix1; ++ix) {
			int cell = base + ix;
			uint64_t mask = (1ULL << (cell & 63));
			if (bits[cell >> 6] & mask) occ_inner++;
		}
	}
	out.occ_inner = occ_inner;

	out.coverage = (float)occ_total / (float)cells;
	out.inner_ratio = (float)occ_inner / (float)occ_total;

	// Final score: spread out + filled interior
	out.score = out.coverage * out.inner_ratio;
	return out;
}


int CPS_ProcessFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr input, bool isLeftSensor, pcl::PointXYZ& refChassisPosition, std::string savePath = std::string(""), int LaneNumber = 0)
{
	// ---- Tunables (relaxed for robustness) ----
	float ROI_X_Min = -2000.f, ROI_X_Max = 2000.f;
	float ROI_Y_Min = -1500.f, ROI_Y_Max = -500.f;
	float ROI_Z_Min = 2200.f, ROI_Z_Max = 4000.f;

	if (isLeftSensor) { ROI_X_Max = -900.f; ROI_X_Min = ROI_X_Max - 800; }
	else { ROI_X_Min = 900.f; ROI_X_Max = ROI_X_Min + 800; }

	float VOX_MM = 30.0f;
	int   SOR_K = 30;
	float SOR_STD = 1.0f;        // previously 1.5

	//SLICE_SCORE
	float SCORE_MIN = 0.15;
	float COVERAGE_MIN = 0.18;
	float INNER_RAIO_MIN = 0.80;
	int CONSEC_K = 2;

	float RW_THRESHOLD = 0.7;
	float RH_THRESHOLD = 0.9;

	int pcThreshold = 200;

	int ChassisPosition = 10000;
	//pcl::PointXYZ refChassisPosition;

	try
	{
		//Logic
		//1. PassThrough Filter (ROI)
		pcl::PointCloud<pcl::PointXYZ>::Ptr pass_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		pc_passThrough(input, ROI_X_Min, ROI_X_Max, ROI_Y_Min, ROI_Y_Max, ROI_Z_Min, ROI_Z_Max, pass_filtered);

		if (savePath != std::string(""))
			if (pass_filtered->points.size() > 0) pcl::io::savePCDFileBinaryCompressed(savePath + "/cps_passThrough.pcd", *pass_filtered);
		
		//2. Voxel DownSample
		pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		//auto v_time = std::chrono::high_resolution_clock::now();
		pc_VoxelDown(pass_filtered, VOX_MM, VOX_MM, VOX_MM, voxel_filtered);
		//logMessage("Voxel Time: " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - v_time).count()) + "ms");

		if (savePath != std::string(""))
			if (voxel_filtered->points.size() > 0) pcl::io::savePCDFileBinaryCompressed(savePath + "/cps_voxelFiltered.pcd", *voxel_filtered);

		//3. Statistical Outlier Removal
		pcl::PointCloud<pcl::PointXYZ>::Ptr sor_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		//auto sor_time = std::chrono::high_resolution_clock::now();
		pc_NoiseFilter(voxel_filtered, SOR_K, SOR_STD, sor_filtered);
		//logMessage("SOR Time: " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - sor_time).count()) + "ms");

		if (savePath != std::string(""))
			if (sor_filtered->points.size() > 0) pcl::io::savePCDFileBinaryCompressed(savePath + "/cps_sorFiltered.pcd", *sor_filtered);

		int baseMinZ, baseMaxZ;
		int baseMinX, baseMaxX;
		int baseMinY, baseMaxY;

		//auto coc_time = std::chrono::high_resolution_clock::now();
		get_center_of_cloud(*sor_filtered, ref(baseMaxX), ref(baseMinX), ref(baseMaxY), ref(baseMinY), ref(baseMaxZ), ref(baseMinZ));
		//logMessage("getCenter Time: " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - coc_time).count()) + "ms");

		//2026.01.06 Added -- use only top 2/3 of the cluster.
		pcl::PointCloud<pcl::PointXYZ>::Ptr pass_filtered_y(new pcl::PointCloud<pcl::PointXYZ>);
		float sMaxY = (float)baseMaxY;
		float sMinY = sMaxY - (sMaxY - (float)baseMinY) * 2 / 3;
		//logMessage("y slice: " + std::to_string(sMinY) + "," + std::to_string(sMaxY));
		pc_passThrough(sor_filtered, sMinY, sMaxY, "y", pass_filtered_y);

		if (savePath != std::string(""))
			if (pass_filtered_y->points.size() > 0) pcl::io::savePCDFileBinaryCompressed(savePath + "/cps_passThrough_on_y.pcd", *pass_filtered_y);

		//Re calc base.
		get_center_of_cloud(*pass_filtered_y, ref(baseMaxX), ref(baseMinX), ref(baseMaxY), ref(baseMinY), ref(baseMaxZ), ref(baseMinZ));

		int baseWidth = abs(baseMaxX - baseMinX);
		int baseHeight = abs(baseMaxY - baseMinY);

		//4. Slice from minZ to maxZ in 50mm slices to get valid tailEnd of the chassis.
		int minZ, maxZ;
		minZ = get_valid_min_z(pass_filtered_y);
		//get_max_min_z(sor_filtered, ref(maxZ), ref(minZ));
		int slice_step = 50;

		int valid_score_slice_count = 0;

		int selected_i = -1;

		int maxPC_slice = 0;

		//Backup.
		int maxPC_iter = -1;
		int maxPC = 0;
		pcl::PointCloud<pcl::PointXYZ>::Ptr maxPCslice(new pcl::PointCloud<pcl::PointXYZ>);

		//minZ to 300mm (10 slices?)
		for (int i = 0; i < 20; i++)
		{
			//instead of 50mm increments, go through 25mm increments but looking at 50mm slice width. 
			pcl::PointCloud<pcl::PointXYZ>::Ptr slice(new pcl::PointCloud<pcl::PointXYZ>);
			int minimumZ = minZ + i * slice_step / 2;
			int maximumZ = minimumZ + slice_step;
			pc_passThrough(pass_filtered_y, minimumZ, maximumZ, "z", slice);

			int refMinZ, refMaxZ;
			int refMinX, refMaxX;
			int refMinY, refMaxY;
			get_center_of_cloud(*slice, ref(refMaxX), ref(refMinX), ref(refMaxY), ref(refMinY), ref(refMaxZ), ref(refMinZ));
			int refWidth = abs(refMaxX - refMinX);
			int refHeight = abs(refMaxY - refMinY);
			
			float ratioW = float(refWidth) / float(baseWidth);
			float ratioH = float(refHeight) / float(baseHeight);

			struct DistScore2 {
				float score = 0.f;       // higher = better distribution
				float coverage = 0.f;    // occ_total / total_cells
				float inner_ratio = 0.f; // occ_inner / occ_total
				int occ_total = 0;
				int occ_inner = 0;
				std::size_t n = 0;
			};

			//auto di_start = std::chrono::high_resolution_clock::now();
			auto score = distributeness_innerfill_fast(slice);
			//logMessage("DI time: " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - di_start).count()) + "ms");
			bool score_valid = (score.coverage >= COVERAGE_MIN && score.inner_ratio >= INNER_RAIO_MIN && score.score >= SCORE_MIN);
			if (LaneNumber != 0)
			{
				RDMessage("Index: " + std::to_string(i) + " Score valid : " + std::to_string(score_valid) + ", cov : " + std::to_string(score.coverage) + ", innerRatio : " + std::to_string(score.inner_ratio) + ", score : " + std::to_string(score.score) + ", pc: " + std::to_string(slice->points.size()) + ", rW: " + std::to_string(ratioW) + ", rH: " + std::to_string(ratioH), LaneNumber);
			}
			else
			{
				logMessage("Index: " + std::to_string(i) + " Score valid : " + std::to_string(score_valid) + ", cov : " + std::to_string(score.coverage) + ", innerRatio : " + std::to_string(score.inner_ratio) + ", score : " + std::to_string(score.score) + ", pc: " + std::to_string(slice->points.size()) + ", rW: " + std::to_string(ratioW) + ", rH: " + std::to_string(ratioH));
			}
			if (slice->points.size() >= pcThreshold && (ratioW > RW_THRESHOLD && ratioH > RH_THRESHOLD) && score_valid)
			{
				if (savePath != std::string(""))
				{
					if (slice->points.size() > 0) pcl::io::savePCDFileBinaryCompressed(savePath + "/cps_selected_slice_z_i_" + std::to_string(i) + "pc_" + std::to_string(slice->points.size()) + "_rW" + std::to_string(ratioW) + "_rH" + std::to_string(ratioH) + ".pcd", *slice);
					if (slice->points.size() > 0) pcl::io::savePCDFileBinaryCompressed(savePath + "/cps_selected_slice_z_i_" + std::to_string(i) + "bW" + std::to_string(baseWidth) + "_bH" + std::to_string(baseHeight) + "_rW" + std::to_string(refWidth) + "_rH" + std::to_string(refHeight) + ".pcd", *slice);
				}

				selected_i = i;

				//refMinZ = get_valid_min_z(slice);
				get_max_min_z(slice, refMaxZ, refMinZ);
				
				ChassisPosition = refMinZ;
				refChassisPosition = pcl::PointXYZ((refMaxX + refMinX) / 2, (refMaxY + refMinY) / 2, refMinZ);

				if (savePath != std::string(""))
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr cpos(new pcl::PointCloud<pcl::PointXYZ>);
					cpos->points.push_back(refChassisPosition);

					if (cpos->points.size() > 0) pcl::io::savePCDFileBinaryCompressed(savePath + "/cps_chassis_position.pcd", *cpos);
				}
				break;
			}
			else if (slice->points.size() > maxPC && (ratioW > (RW_THRESHOLD - 0.05) && ratioH > (RH_THRESHOLD - 0.05)))
			{
				maxPC_iter = i;
				maxPC = slice->points.size();
				pcl::copyPointCloud(*slice, *maxPCslice);

				if (savePath != std::string(""))
					if (slice->points.size() > 0) pcl::io::savePCDFileBinaryCompressed(savePath + "/cps_slice_z_i_" + std::to_string(i) + "pc_" + std::to_string(slice->points.size()) + "_rW" + std::to_string(ratioW) + "_rH" + std::to_string(ratioH) + ".pcd", *slice);
			}
			else
			{
				if (savePath != std::string(""))
					if (slice->points.size() > 0) pcl::io::savePCDFileBinaryCompressed(savePath + "/cps_slice_z_i_" + std::to_string(i) + "pc_" + std::to_string(slice->points.size()) + "_rW" + std::to_string(ratioW) + "_rH" + std::to_string(ratioH) + ".pcd", *slice);

			}
		}

		
		if (selected_i == -1 && maxPCslice->points.size() > 100)
		{
			if (savePath != std::string(""))
			{
				if (maxPCslice->points.size() > 0) pcl::io::savePCDFileBinaryCompressed(savePath + "/cps_backup_selected_slice_z_i_" + std::to_string(maxPC_iter) + "pc_" + std::to_string(maxPCslice->points.size()) +".pcd", *maxPCslice);
			}
			//refMinZ = get_valid_min_z(slice);

			int refMinZ, refMaxZ;
			int refMinX, refMaxX;
			int refMinY, refMaxY;
			get_center_of_cloud(*maxPCslice, ref(refMaxX), ref(refMinX), ref(refMaxY), ref(refMinY), ref(refMaxZ), ref(refMinZ));

			get_max_min_z(maxPCslice, refMaxZ, refMinZ);

			ChassisPosition = refMinZ;
			refChassisPosition = pcl::PointXYZ((refMaxX + refMinX) / 2, (refMaxY + refMinY) / 2, refMinZ);

			if (savePath != std::string(""))
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr cpos(new pcl::PointCloud<pcl::PointXYZ>);
				cpos->points.push_back(refChassisPosition);

				if (cpos->points.size() > 0) pcl::io::savePCDFileBinaryCompressed(savePath + "/cps_backup_chassis_position.pcd", *cpos);
			}
		}
		
		return ChassisPosition;
		
	}
	catch (std::exception& ex)
	{
		logMessage("[CPS_ProcessFrame] " + std::string(ex.what()));
		return -1;
	}
	catch (...)
	{
		logMessage("[CPS_ProcessFrame] Unknown Exception!");
		return -1;
	}
}

//CPS Thread
void CPS_Processing_Thread(int LaneNumber)
{
	try
	{
		logMessage("CPS Processing -- Thread " + std::to_string(LaneNumber) + " Activated");

		int procIndex = LaneNumber - 1;
		if (procIndex < 0) { logMessage("Wrong Proc Number!"); return; }

		while (cps_running[procIndex].load())
		{
			std::unique_lock<std::mutex> lock(mutex_cps[procIndex]);
			bool res = cond_cps[procIndex].wait_for(lock,
				std::chrono::seconds(3600),
				[procIndex]() { return cps_flag[procIndex]; });

			if (res)
			{
				logMessage("Enabled CPS Processing -- Thread " + std::to_string(LaneNumber));
				logMessage("Assigned Lane: " + std::to_string(LaneNumber));

				RDMessage("Enabled CPS Processing -- Thread " + std::to_string(LaneNumber), LaneNumber);
				RDMessage("Assigned Lane: " + std::to_string(LaneNumber), LaneNumber);

				isRunning[procIndex] = true;
				CPS_Lane_Enabled[procIndex] = true;

				//Visionary T Mini Setup.
				using namespace visionary;
				std::string deviceIpAddr = "";
				unsigned short deviceBlobCtrlPort = 2114;
				unsigned cnt = 100u;

				controlConnected[procIndex] = false;
				dataConnected[procIndex] = false;

				while (CPS_Lane_Enable[LaneNumber - 1] && !CPS_Lane_Completed[LaneNumber - 1])
				{
					bool isLeftSensor = true;
					
					//IP Settings.
					if (Active_Job_Lane == 0)
					{
						//Default sensor is "Left" Sensor
						deviceIpAddr = IP_ADDRESSES[LaneNumber];	
					}
					else
					{
						if (LaneNumber + 1 == Active_Job_Lane)
						{
							//Right sensor
							deviceIpAddr = IP_ADDRESSES[LaneNumber -1];
							isLeftSensor = false;
						}
						else
						{
							//Left Sensor -- default
							deviceIpAddr = IP_ADDRESSES[LaneNumber];
						}
					}

					//Connect to sensor
					if (!visionaryControl[procIndex].open(VisionaryControl::ProtocolType::COLA_2, deviceIpAddr, 3000/*ms*/))
					{
						logMessage("Failed to open control connection to device.");
					}
					else
					{
						logMessage("Control stream opened.");
						controlConnected[procIndex] = true;
						//-----------------------------------------------
						// read Device Ident
						logMessage("DeviceIdent: " + visionaryControl[procIndex].getDeviceIdent());
						if (!visionaryControl[procIndex].logout())
						{
							logMessage("Failed to logout");
						}

						if (!dataStream[procIndex].open(deviceIpAddr, htons(deviceBlobCtrlPort)))
						{
							logMessage("Failed to open data stream connection to device.");
						}
						else
						{
							dataConnected[procIndex] = true;
							visionaryControl[procIndex].stopAcquisition();

							logMessage("Data Stream opened.");

						}
					}

					//check for sensor connections -- fault if needed.
					if (!controlConnected[procIndex] || !dataConnected[procIndex])
					{
						logMessage("Failed to connect to sensor for Lane: " + std::to_string(LaneNumber) + " at ip: " + deviceIpAddr);

						CPS_Lane_Fault[LaneNumber - 1] = -10001;
						CPS_Lane_RD[LaneNumber - 1] = -10001;
						CPS_Lane_Completed[LaneNumber - 1] = true;

						clear_proc_status(procIndex + 1);

						break;
					}

					//Another while loop for processing
					auto last_frame_get_time = std::chrono::system_clock::now();

					int save_count_current_bucket[5] = { 0, 0, 0, 0, 0 };
					int current_bucket_number = 0;

					int Complete_Count = 0;

					auto procStarted_time = std::chrono::system_clock::now();
					
					bool saveTriggered = false;
					int triggered_bucket_index = -1;

					//trigger when valid data got, then 10000 appeared.
					bool invalid_frame_save_trigger = false;
					bool got_valid_output = false;

					while (true)
					{
						if (!CPS_Lane_Enable[procIndex]) break;
						if (activeLaneChanged[procIndex]) 
						{
							activeLaneChanged[procIndex] = false;
							break;
						}
						if (CPS_Lane_Completed[procIndex])
						{
							//reset proc values?
							clear_proc_status(procIndex + 1);
							break;
						}
						if (CPS_Lane_End_Signal[procIndex])
						{
							logMessage("CPS Lane " + std::to_string(LaneNumber) + " Marked as Ended. Stopping CPS process.");
							CPS_Lane_Completed[procIndex] = true;
							clear_proc_status(LaneNumber);
							break;
						}

						//1min -> 10min.
						if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - procStarted_time).count() > 600000)
						{
							logMessage("CPS Lane " + std::to_string(LaneNumber) + " Running past 1 Minute. Stopping CPS process.");
							CPS_Lane_Completed[procIndex] = true;
							clear_proc_status(LaneNumber);
							break;
						}

						auto waitDur_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - last_frame_get_time).count();
						if (waitDur_ms > 100)
						{
							bool stepComplete = visionaryControl[procIndex].stepAcquisition();
							if (stepComplete)
							{
								if (dataStream[procIndex].getNextFrame())
								{
									blnGetNewFrame = true;
									last_frame_get_time = std::chrono::system_clock::now();

									{
										//-----------------------------------------------
										// Convert data to a point cloud
										std::vector<PointXYZ> pointCloud;
										pDataHandler[procIndex]->generatePointCloud(pointCloud);
										pDataHandler[procIndex]->transformPointCloud(pointCloud);

										auto intensityMap = pDataHandler[procIndex]->getIntensityMap();

										auto iW = pDataHandler[procIndex]->getWidth();
										auto iH = pDataHandler[procIndex]->getHeight();
										auto gImg = cv::Mat(pDataHandler[procIndex]->getHeight(), pDataHandler[procIndex]->getWidth(), CV_16UC1, intensityMap.data());
										cv::Mat im3; // I want im3 to be the CV_16UC1 of im2
										gImg.convertTo(im3, CV_8UC1);

										cv::Mat grayToClr;
										cv::cvtColor(im3, grayToClr, cv::COLOR_GRAY2BGR);

										auto pclPointCloud = makePCL_PointCloud(pointCloud);
										if (pclPointCloud == NULL)
										{
											logMessage("[TMini-Data-Stream] Failed to create PCL Point Cloud from data handler. -- nullptr!");
											continue;
										}

										/*
										CPSFrameResult cpsFrameResult;
										cpsFrameResult = processCPSFrame(pclPointCloud, isLeftSensor, std::string(""), std::string(""), false);

										int chassisPos = 10000;
										if (cpsFrameResult.tailPoints.size() > 0)
										{
											chassisPos = cpsFrameResult.tailPoints.at(0).z;
										}
										*/
										//grayToClr, pclPointCloud.
										pcl::PointXYZ refChassisPosition(0, 0, 0);
										int chassisPos = CPS_ProcessFrame(pclPointCloud, isLeftSensor, refChassisPosition, std::string(""), LaneNumber);
										
										/*
										auto temp = chassisPos - TARGET_STOP_POSITION_MM;
										if (abs(temp - CPS_Lane_RD[LaneNumber - 1]) < 200 && chassisPos < 3000)
										{
											if (Complete_Count < 5) Complete_Count++;
											if (Complete_Count >= 5)
											{
												if (!CPS_Lane_Completed[LaneNumber - 1])
												{
													CPS_Lane_Completed[LaneNumber - 1] = true;
													logMessage("CPS Lane " + std::to_string(LaneNumber) + " Chassis not moving. Marking as Completed.");													
												}
											}
										}
										*/
										CPS_Lane_RD[LaneNumber - 1] = chassisPos;// -TARGET_STOP_POSITION_MM;
										RDMessage("[CPS-Lane: " + std::to_string(LaneNumber) + "] Chassis Position(MinZ) : " + std::to_string(chassisPos) + " mm" + " , RD: " + std::to_string(CPS_Lane_RD[LaneNumber- 1]), LaneNumber, true);
										
										invalid_frame_save_trigger = false;
										if (chassisPos != 10000)
										{
											got_valid_output = true;
										}
										else
										{
											if (got_valid_output) invalid_frame_save_trigger = true;
										}



										//BUCKET COUNT
										bool saveEnable = false;
										if (ENABLE_SAVE_ALL) saveEnable = true;

										if ((chassisPos != 10000 || saveTriggered || invalid_frame_save_trigger) && !saveEnable)
										{
											
											if (chassisPos == 10000)
											{
												if (invalid_frame_save_trigger)
												{
													got_valid_output = false;
												}

												//save 5 here.
												if (save_count_current_bucket[4] < 10)
												{
													if (!saveTriggered) { saveTriggered = true; triggered_bucket_index = 4; }
													saveEnable = true;
													save_count_current_bucket[4]++;
												}

												else
												{
													saveTriggered = false;
													triggered_bucket_index = -1;
												}
											}
											else if (chassisPos > 3000 && chassisPos <= 3500)
											{
												//save 5 here.
												if (save_count_current_bucket[0] < 5)
												{
													if (!saveTriggered) { saveTriggered = true; triggered_bucket_index = 0; }
													saveEnable = true;
													save_count_current_bucket[0]++;
												}

												else
												{
													saveTriggered = false;
													triggered_bucket_index = -1;
												}
											}
											else if (chassisPos > 2500 && chassisPos <= 3000)
											{
												if (save_count_current_bucket[1] < 5)
												{
													if (!saveTriggered) { saveTriggered = true; triggered_bucket_index = 1; }
													saveEnable = true;
													save_count_current_bucket[1]++;
												}
												else
												{
													saveTriggered = false;
													triggered_bucket_index = -1;
												}
											}
											else if (chassisPos > 2200 && chassisPos <= 2500)
											{
												if (save_count_current_bucket[2] < 5)
												{
													if (!saveTriggered) { saveTriggered = true; triggered_bucket_index = 2; }
													saveEnable = true;
													save_count_current_bucket[2]++;
												}
												else
												{
													saveTriggered = false;
													triggered_bucket_index = -1;
												}
											}
											else
											{
												if (save_count_current_bucket[3] < 5)
												{
													if (!saveTriggered) { saveTriggered = true; triggered_bucket_index = 3; }
													saveEnable = true;
													save_count_current_bucket[3]++;
												}
												else
												{
													saveTriggered = false;
													triggered_bucket_index = -1;
												}
											}
										}
										if (saveEnable)
										{
											//Push to save queue.
											auto dataTup = std::tuple<cv::Mat, pcl::PointCloud<pcl::PointXYZ>, bool, int, std::string, std::chrono::system_clock::time_point>(grayToClr, *pclPointCloud, isLeftSensor, procIndex, std::string(""), last_frame_get_time);
											tsq.push(dataTup);
										}

										//For testing only
										/*
										if (save_count_current_bucket[3] >= 5)
										{
											if (!CPS_Lane_Completed[LaneNumber - 1])
											{
												CPS_Lane_Completed[LaneNumber - 1] = true;
												logMessage("CPS Lane " + std::to_string(LaneNumber) + " Reached minimum save count. Marking as Completed.");						
											}
										}
										*/

										//Final Completion Check.
										/*
										if (abs(CPS_Lane_RD[LaneNumber - 1] - 200))
										{
											if (Complete_Count++ > 10)
											{
												CPS_Lane_Completed[LaneNumber - 1] = true;
												logMessage("CPS Lane " + std::to_string(LaneNumber) + " Completed!");
											}
										}
										else
										{
											if (Complete_Count > 0) Complete_Count--;
										}
										*/

									}
								}
							}

							std::this_thread::sleep_for(std::chrono::milliseconds(1));
						}
						else
						{
							std::this_thread::sleep_for(std::chrono::milliseconds(5));
						}

					}

					//disconnect here.
					visionaryControl[procIndex].close();
					dataStream[procIndex].close();

					logMessage("Disconnected from current sensor");

					this_thread::sleep_for(std::chrono::milliseconds(50));
				}

				isRunning[procIndex] = false;
			}

			else
			{
				CPS_Lane_Enabled[procIndex] = false;
			}
		}
	}
	catch (std::exception& ex)
	{
		logMessage("[CPS-Processing] " + std::string(ex.what()));
	}
	catch (...)
	{
		logMessage("[CPS-Processing] Unknown Exception!");
	}
}

void clear_proc_status()
{
	for (int i = 0; i < 6; i++)
	{
		cps_flag[i] = false;

		isAssigned[i] = false;
		//assignedLaneNumber[i] = -1;
		
		isAvailable[i] = true;
		isRunning[i] = false;
		activeLaneChanged[i] = false;
	}

	while (!cps_queue.empty())
	{
		cps_queue.pop();
	}

	for (int i = 0; i < 6; i++)
	{
		assignedProcNumber[i] = -1;
		CPS_Lane_Completed[i] = false;
		CPS_Lane_Enable[i] = false;
		CPS_Lane_Enabled[i] = false;
		CPS_Lane_Fault[i] = -10000;
		//CPS_Lane_RD[i] = -10000;
	}

	logMessage("CPS Status Cleared");
}

//CPS Debugging
void CPS_Processing_Debug()
{
	logMessage("Debugging CPS Logic");

	auto jobDirectories = ListSubDirectories(DEBUG_BATCH_ROOT_DIR);
	bool savePLY = true;

	for (const auto& jobDir : jobDirectories)
	{
		logMessage("Processing: " + jobDir);

		auto subDir = getSubdirectoriesName(jobDir);
		if (subDir.size() < 1)
		{
			logMessage("No subdirectories found in job directory!");
			continue;
		}

		auto subDirectionDir = getSubdirectoriesName(jobDir + "/" + subDir[0]);
		if (subDirectionDir.size() < 1)
		{
			logMessage("No direction subdirectories found in job directory!");
			continue;
		}
		
		auto isLeftSensor = (subDirectionDir[0] == "LeftSensor") ? true : false;

		//load .jpg, .pcd files for processing
		auto image_filePath = jobDir + "/" + subDir[0] + "/" + subDirectionDir[0] + "/" + std::string("Image");
		auto depth_filePath = jobDir + "/" + subDir[0] + "/" + subDirectionDir[0] + "/" + std::string("Depth");


		std::filesystem::path pathObj = std::filesystem::path(jobDir).lexically_normal();
		std::string lastDirectory = pathObj.filename().string();

		auto image_files = getAllFiles(image_filePath, ".jpg");
		auto depth_files = getAllFiles(depth_filePath, ".ply");
		if (depth_files.size() == 0) depth_files = getAllFiles(depth_filePath, ".pcd");

		createDirectory_ifexists(DEBUG_BATCH_SAVE_DIR);
		auto save_file_path = DEBUG_BATCH_SAVE_DIR + "/" + lastDirectory;
		auto folder_created = createDirectory_ifexists(save_file_path);

		if (!folder_created)
		{
			logMessage("Skipping already existing folder: " + save_file_path);
			continue;
		}

		logMessage("Saving to " + save_file_path);

		auto logHeader = std::string("Filename;TargetDistance");

		ofstream Simfile;
		std::string sim_result_txt = save_file_path + "/" + lastDirectory + "_res.txt";
		Simfile.open(sim_result_txt.c_str(), ios::out | ios::app);
		if (!Simfile.is_open())
		{
			logMessage("Failed to open file?");
		}
		if (Simfile.is_open())
		{
			Simfile << logHeader + "\n";
			Simfile.close();
		}

		for (int i = 0; i < depth_files.size(); i++)
		{
			logMessage("Processing Cycle Start!");

			std::filesystem::path pathObj(depth_files.at(i));
			// Get the filename with extension
			std::string filename = pathObj.stem().string();

			std::string log_lines = filename + ";";

			auto save_current_file_path = save_file_path + "/" + filename;
			createDirectory_ifexists(save_current_file_path);

			//load ply
			bool loadPLYFail = false;
			pcl::PointCloud<pcl::PointXYZ>::Ptr pcData = std::make_shared<pcl::PointCloud <pcl::PointXYZ>>();
			if (pcl::io::loadPLYFile<pcl::PointXYZ>(depth_files.at(i), *pcData) == -1)
			{
				std::cout << std::string("Couldn't read file by PLY ") << std::endl;
				loadPLYFail = true;
				//return false;
			}
			//load ply
			//pcl::PointCloud<pcl::PointXYZ>::Ptr pcData = std::make_shared<pcl::PointCloud <pcl::PointXYZ>>();
			if (loadPLYFail)
			{
				if (pcl::io::loadPCDFile<pcl::PointXYZ>(depth_files.at(i), *pcData) == -1)
				{
					std::cout << std::string("Couldn't read file by PCD ") << std::endl;
					//return false;
				}
			}
			auto pointCloud = makePCL_PointCloud(*pcData, DEBUG_CONVERT_PCL_RANGE);
			pcl::io::savePCDFileBinaryCompressed(save_current_file_path + "/" + filename + "_base_converted.pcd", *pointCloud);

			auto savePath = save_current_file_path + "/" + filename;

			pcl::PointXYZ refChassisPosition(0, 0, 0);
			int chassisPos = CPS_ProcessFrame(pointCloud, isLeftSensor, refChassisPosition, save_current_file_path);

			log_lines += std::to_string(chassisPos);// +"\n";

			Simfile.open(sim_result_txt.c_str(), ios::out | ios::app);
			if (!Simfile.is_open())
			{
				logMessage("Failed to open file?");
			}
			if (Simfile.is_open())
			{
				Simfile << log_lines + "\n";
				Simfile.close();
			}
		}

	}


}

void handleClient(SOCKET clientSocket) {
	logMessage("Client connected. Sending initial message...");

	char buffer[DEFAULT_RECVLEN] = { 0 };
	while (socket_running.load()) 
	{
		int bytesReceived = recv(clientSocket, buffer, sizeof(buffer), 0);
		if (bytesReceived <= 0) {
			logMessage("Client disconnected. Waiting for new client...");
			break;
		}
		//buffer[bytesReceived] = '\0';

		parseCommand(buffer);

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		
		char sendbuf[DEFAULT_SENDLEN] = {};
		makeCommand(ref(sendbuf));

		int bytesSent = send(clientSocket, sendbuf, DEFAULT_SENDLEN, 0);
		if (bytesSent <= 0)
		{
			logMessage("Sending data failed!");
			break;
		}
	}
	closesocket(clientSocket);
}
//2025.02.06 Updated socket server
int start_server() {
	WSADATA wsaData;
	SOCKET serverSocket;
	struct sockaddr_in serverAddr;
	int addrlen = sizeof(serverAddr);

	// Initialize Winsock
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
		std::cerr << "WSAStartup failed!\n";
		logMessage("WSAStartup failed!");
		return 1;
	}

	// Create socket
	serverSocket = socket(AF_INET, SOCK_STREAM, 0);
	if (serverSocket == INVALID_SOCKET) {
		std::cerr << "Socket creation failed!\n";
		logMessage("Socket creation failed!");
		WSACleanup();
		return 1;
	}

	BOOL on = TRUE;
	if (setsockopt(serverSocket, SOL_SOCKET, SO_EXCLUSIVEADDRUSE,
		reinterpret_cast<const char*>(&on), sizeof(on)) == SOCKET_ERROR) {
		std::cerr << "Socket reuse port failed!\n";
		logMessage("Socket reuse port failed!");
		WSACleanup();
		return 1;
	}


	// Bind socket
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_addr.s_addr = INADDR_ANY;
	serverAddr.sin_port = htons(SOCKET_PORT);

	if (::bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
		std::cerr << "Bind failed!\n";
		logMessage("Bind failed!");
		closesocket(serverSocket);
		WSACleanup();
		return 1;
	}

	// Start listening
	if (listen(serverSocket, SOMAXCONN) == SOCKET_ERROR) {
		std::cerr << "Listen failed!\n";
		logMessage("Listen failed");
		closesocket(serverSocket);
		WSACleanup();
		return 1;
	}

	logMessage("Server listening on port " + std::to_string(SOCKET_PORT) + "...");

	while (socket_running.load()) 
	{
		try
		{
			struct sockaddr_in clientAddr;
			int clientSize = sizeof(clientAddr);
			SOCKET clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &clientSize);

			if (clientSocket == INVALID_SOCKET)
			{
				logMessage("Accept failed!");
				std::cerr << "Accept failed!\n";
				continue;
			}

			// Handle client in a separate thread
			std::thread clientThread(handleClient, clientSocket);

			clientThread.join();  // Wait for the thread to finish

			clear_proc_status();
			//clientThread.detach();  // Let it run independently
			logMessage("Waiting for new client...");
		}
		catch (std::exception& ex)
		{
			logMessage("[SERVER] " + std::string(ex.what()));
		}
		catch (...)
		{
			logMessage("[SERVER] Unknown Exception!");
		}
	}

	logMessage("Terminating Socket Server");
	closesocket(serverSocket);
	WSACleanup();
	return 0;
}

bool runStreamingDemo(const char ipAddress[], unsigned short dataPort, uint32_t numberOfFrames, bool executeExtTrigger)
{
	using namespace visionary;

	// Generate Visionary instance
	auto pDataHandler = std::make_shared<VisionaryTMiniData>();
	VisionaryDataStream dataStream(pDataHandler);
	VisionaryControl visionaryControl;

	std::printf("Made samples.\n");
	std::cout << ipAddress << " ;;; " << dataPort << std::endl;

	//-----------------------------------------------
	// Connect to devices data stream 
	if (!dataStream.open(ipAddress, htons(dataPort)))
	{
		std::printf("Failed to open data stream connection to device.\n");
		return false;   // connection failed
	}

	std::printf("Data stream opened.\n");

	//-----------------------------------------------
	// Connect to devices control channel
	if (!visionaryControl.open(VisionaryControl::ProtocolType::COLA_2, ipAddress, 5000/*ms*/))
	{
		std::printf("Failed to open control connection to device.\n");
		return false;   // connection failed
	}

	std::printf("Control stream opened.\n");


	//-----------------------------------------------
	// read Device Ident
	std::printf("DeviceIdent: '%s'\n", visionaryControl.getDeviceIdent().c_str());

	//-----------------------------------------------
	// Login as authorized client
	if (visionaryControl.login(IAuthentication::UserLevel::AUTHORIZED_CLIENT, "CLIENT"))
	{
		//-----------------------------------------------
		// An example of reading an writing device parameters is shown here.
		// Use the "SOPAS Communication Interface Description" PDF to determine data types for other variables
		//-----------------------------------------------
		// Set enDepthMask parameter to false

		std::printf("Setting enDepthMask to false\n");
		CoLaCommand setEnDepthMaskCommand = CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "enDepthMask").parameterBool(false).build();
		CoLaCommand setEnDepthMaskResponse = visionaryControl.sendCommand(setEnDepthMaskCommand);
		if (setEnDepthMaskResponse.getError() == CoLaError::OK)
		{
			std::printf("Successfully set enDepthMask to false\n");
		}


		//-----------------------------------------------
		// Read humidity parameter
		CoLaCommand getHumidity = CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "humidity").build();
		CoLaCommand humidityResponse = visionaryControl.sendCommand(getHumidity);
		const double humidity = CoLaParameterReader(humidityResponse).readLReal();
		std::printf("Read humidity = %f\n", humidity);

		//-----------------------------------------------
		// Read info messages variable
		CoLaCommand getMessagesCommand = CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "MSinfo").build();
		CoLaCommand messagesResponse = visionaryControl.sendCommand(getMessagesCommand);

		//-----------------------------------------------
	}

	{
		CoLaCommand setEnDepthMaskCommand = CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "enDepthMask").parameterBool(true).build();
		CoLaCommand setEnDepthMaskResponse = visionaryControl.sendCommand(setEnDepthMaskCommand);
		if (setEnDepthMaskResponse.getError() != CoLaError::OK)
		{
			std::printf("Failed to set enDepthMask to true\n");
		}
	}

	//-----------------------------------------------
	// Logout from device after reading variables.
	if (!visionaryControl.logout())
	{
		std::printf("Failed to logout\n");
	}

	//-----------------------------------------------
	// Stop image acquisition (works always, also when already stopped)
	visionaryControl.stopAcquisition();

	//-----------------------------------------------
	// Capture a single frame
	visionaryControl.stepAcquisition();
	if (dataStream.getNextFrame())
	{
		std::printf("Frame received through step called, frame #%d, timestamp: %u \n", pDataHandler->getFrameNum(), pDataHandler->getTimestampMS());

		//-----------------------------------------------
		// Convert data to a point cloud
		std::vector<PointXYZ> pointCloud;
		pDataHandler->generatePointCloud(pointCloud);
		pDataHandler->transformPointCloud(pointCloud);



		//-----------------------------------------------
		// Write point cloud to PLY
		const char plyFilePath[] = "VisionaryT.pcd";
		std::printf("Writing frame to %s\n", plyFilePath);
		PointCloudPlyWriter::WriteFormatPLY(plyFilePath, pointCloud, pDataHandler->getIntensityMap(), true);
		std::printf("Finished writing frame to %s\n", plyFilePath);
	}

	//-----------------------------------------------
	// Start image acquisiton and continously receive frames
	visionaryControl.startAcquisition();
	for (uint32_t i = 0; i < numberOfFrames; i++)
	{
		if (!dataStream.getNextFrame())
		{
			continue;     // No valid frame received
		}
		std::printf("Frame received in continuous mode, frame #%d \n", pDataHandler->getFrameNum());
		std::vector<uint16_t> intensityMap = pDataHandler->getIntensityMap();
		std::vector<uint16_t> distanceMap = pDataHandler->getDistanceMap();
		std::vector<uint16_t> stateMap = pDataHandler->getStateMap();
	}

	//-----------------------------------------------
	// This part of the sample code is skipped by default because not every user has a working IO trigger hardware available. 
	// If you want to execute it set variable "executeExtTrigger" in main function to "true".
	if (executeExtTrigger)
	{
		// Capture single frames with external trigger
		// NOTE: This part of the sample only works if you have a working rising egde signal on IO1 which triggers an image!
		std::printf("\n=== Starting external trigger example: \n");
		// Login as authorized client
		if (visionaryControl.login(IAuthentication::UserLevel::AUTHORIZED_CLIENT, "CLIENT"))
		{
			// Set frontendMode to STOP (= 1)
			std::printf("Setting frontendMode to STOP (= 1)\n");
			CoLaCommand setFrontendModeCommand = CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "frontendMode").parameterUSInt(1).build();
			CoLaCommand setFrontendModeResponse = visionaryControl.sendCommand(setFrontendModeCommand);
			if (setFrontendModeResponse.getError() != CoLaError::OK)
			{
				std::printf("Failed to set frontendMode to STOP (= 1)\n");
			}

			// Set INOUT1_Function to Trigger (= 7)
			std::printf("Setting DIO1Fnc to Trigger (= 7)\n");
			CoLaCommand setDIO1FncCommand = CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "DIO1Fnc").parameterUSInt(7).build();
			CoLaCommand setDIO1FncResponse = visionaryControl.sendCommand(setDIO1FncCommand);
			if (setDIO1FncResponse.getError() != CoLaError::OK)
			{
				std::printf("Failed to set DIO1Fnc to Trigger (= 7)\n");
			}

			// Set INOUT2_Function to TriggerBusy (= 23)
			std::printf("Setting DIO2Fnc to TriggerBusy (= 23)\n");
			CoLaCommand setDIO2FncCommand = CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "DIO2Fnc").parameterUSInt(23).build();
			CoLaCommand setDIO2FncResponse = visionaryControl.sendCommand(setDIO2FncCommand);
			if (setDIO2FncResponse.getError() != CoLaError::OK)
			{
				std::printf("Failed to set DIO2Fnc to TriggerBusy (= 23)\n");
			}
		}

		// Re-Connect to device data stream (make sure there are no old images in the pipeline)
		dataStream.close();
		std::this_thread::sleep_for(std::chrono::seconds(1)); // This short deelay is necessary to not have any old frames in the pipeline.
		if (!dataStream.open(ipAddress, htons(dataPort)))
		{
			std::printf("Failed to open data stream connection to device.\n");
			return false;   // connection failed
		}

		std::printf("Please enable trigger on IO1 to receive an image: \n");
		long long startTime = std::chrono::system_clock::now().time_since_epoch().count();
		long long timeNow = startTime;

		// Limited time loop for receiving hardware trigger signals on IO1 pin
		bool frameReceived = false;
		long long triggerTimeOut = 100000000; // 10 sec = 100 000 000
		while ((timeNow - startTime) <= triggerTimeOut) {
			// Read variable IOValue
			CoLaCommand getIOValue = CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "IOValue").build();
			CoLaCommand IOValueResponse = visionaryControl.sendCommand(getIOValue);
			CoLaParameterReader IOValues(IOValueResponse);
			const int8_t IOValue1 = IOValues.readSInt();
			const int8_t IOValue2 = IOValues.readSInt(); // We need the IOValue of IO2 from the V3SIOsState struct
			std::printf("Read TriggerBusy = %d\n", IOValue2);

			// Receive the next frame
			if (IOValue2 == 0)
			{
				if (dataStream.getNextFrame())
				{
					std::printf("Frame received in external trigger mode, frame #%d \n", pDataHandler->getFrameNum());
					frameReceived = true;
				}
				timeNow = std::chrono::system_clock::now().time_since_epoch().count();
			}
		}

		if (frameReceived == false)
		{
			std::printf("TIMEOUT: No trigger signal received on IO1 within %.2f seconds!\n", (float)triggerTimeOut / 10000000);
		}
	}
	//-----------------------------------------------

	visionaryControl.close();
	dataStream.close();
	return true;
}

std::string Utf16ToUtf8(const std::wstring& w) {
	int len = WideCharToMultiByte(CP_UTF8, 0, w.data(), w.size(), nullptr, 0, nullptr, nullptr);
	std::string s(len, '\0');
	WideCharToMultiByte(CP_UTF8, 0, w.data(), w.size(), &s[0], len, nullptr, nullptr);
	return s;
}
namespace fs = std::filesystem;
std::string getParent(const std::string& pathStr) {
	fs::path p(pathStr);
	return p.parent_path().string();
}
int main(int argc, char* argv[])
{
	wchar_t buf[MAX_PATH];
	DWORD sz = GetModuleFileNameW(nullptr, buf, MAX_PATH);
	if (sz > 0 && sz < MAX_PATH) {
		std::wstring exePath(buf);
		app_path = Utf16ToUtf8(exePath);
	}

	app_path = getParent(app_path);

	HANDLE mutex = CreateMutexA(nullptr, TRUE, app_path.c_str());

	if (GetLastError() == ERROR_ALREADY_EXISTS) {
		std::cerr << "Another instance is already running.\n";
		return 1;
	}

	SetUnhandledExceptionFilter(MyUnhandledExceptionFilter);

	printf("CPS Application\n");

	parseAppName(app_path);

	appName = appID + "_v" + program_version;
	std::cout << "App: " << appName << "\n";

	std::thread logThread(logWriterThread);
	
	logMessage(appName + " Starting!");
	logMessage(app_path);

	signal(SIGINT, my_handler);
	//std::set_terminate(unHandledExceptionHandler);

	parseINI(app_path);
	parseIPINI(app_path);

	if (DEBUG_WITH_FILES || DEBUG_BATCH_JOB)
	{
		CPS_Processing_Debug();
	}
	else if (MODE_DEBUG)
	{
		logMessage("Debug Mode running.. Testing for onnx.");

		//CURRENT_SENSOR_POSITION = "REAR_LEFT";

		DEBUG_IMG_PATH = DEBUG_SAMPLE_JOB + "/Image";
		DEBUG_PLY_PATH = DEBUG_SAMPLE_JOB + "/Depth";

		DEBUG_IMG_FILES = getAllFiles(DEBUG_IMG_PATH, ".jpg");
		DEBUG_PLY_FILES = getAllFiles(DEBUG_PLY_PATH, ".pcd");

		DEBUG_MAX_INDEX = (DEBUG_IMG_FILES.size() < DEBUG_PLY_FILES.size()) ? DEBUG_IMG_FILES.size() : DEBUG_PLY_FILES.size();
		DEBUG_CURRENT_INDEX = 0;

		/*
		demo_img = cv::imread("20250302_041440_568_TMini_Image.jpg", cv::IMREAD_COLOR);

		pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = std::make_shared<pcl::PointCloud <pcl::PointXYZ>>();
		if (pcl::io::loadPLYFile<pcl::PointXYZ>("20250302_041440_568_TMini_Depth.pcd", *pointCloud) == -1)
		{
			std::cout << std::string("Couldn't read file by PLY ") << std::endl;
			//return false;
		}
		
		demo_ply->points.resize(pointCloud->points.size());

		std::transform(pointCloud->points.begin(), pointCloud->points.end(), demo_ply->points.begin(),
			[](pcl::PointXYZ val) { return pcl::PointXYZ(val.x * 1000, val.y * 1000, val.z * 1000); });
		*/
		//std::thread sckT(start_server);

		//need to specify and enable connection.
		//current_lane_ip = IP_ADDRESSES[0];
		
		//trigger tmini setup and stream.
		//tmini_ctrl_flag = true;
		//cond_tmini_ctrl.notify_one();

		//std::thread tProc(processingThread, std::ref(detector));
		std::thread tSave(data_save_thread);

		//tProc.join();
		tSave.join();
		//sckT.join();
	}
	else
	{
		std::thread sckT(start_server);

		std::thread proc1(CPS_Processing_Thread, 1);
		std::thread proc2(CPS_Processing_Thread, 2);
		std::thread proc3(CPS_Processing_Thread, 3);
		std::thread proc4(CPS_Processing_Thread, 4);
		std::thread proc5(CPS_Processing_Thread, 5);
		std::thread proc6(CPS_Processing_Thread, 6);

		std::thread RDThread1(RDWriterThread, 1);
		std::thread RDThread2(RDWriterThread, 2);
		std::thread RDThread3(RDWriterThread, 3);
		std::thread RDThread4(RDWriterThread, 4);
		std::thread RDThread5(RDWriterThread, 5);
		std::thread RDThread6(RDWriterThread, 6);

		std::thread tSave(data_save_thread);

		logMessage("CPS Threads all running");

		if (sckT.joinable()) sckT.join();
		
		if (proc1.joinable()) proc1.join();
		if (proc2.joinable()) proc2.join();
		if (proc3.joinable()) proc3.join();
		if (proc4.joinable()) proc4.join();
		if (proc5.joinable()) proc5.join();
		if (proc6.joinable()) proc6.join();

		if (tSave.joinable()) tSave.join();

		if (RDThread1.joinable()) RDThread1.join();
		if (RDThread2.joinable()) RDThread2.join();
		if (RDThread3.joinable()) RDThread3.join();
		if (RDThread4.joinable()) RDThread4.join();
		if (RDThread5.joinable()) RDThread5.join();
		if (RDThread6.joinable()) RDThread6.join();

	}

	logMessage("Program Shutting Down...");

	log_running.store(false);
	cvLog.notify_one();
	logThread.join();

	//std::cout << "Press any key to continue...";
	//system("pause"); // Windows only

	return 1;
}