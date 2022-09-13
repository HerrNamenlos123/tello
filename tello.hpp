
#ifndef _TELLO_H
#define _TELLO_H

// #define TELLO_ONLY_DECLARE



// ====================================================================================
// ===                                                                              ===
// ===   Begin of the UDPsocket library (https://github.com/barczynsky/UDPsocket)   ===
// ===                                                                              ===
// ====================================================================================

#ifndef TELLO_ONLY_DECLARE

#pragma once
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#ifndef INPORT_ANY
#define INPORT_ANY 0
#endif

#include <cstring>
#include <array>
#include <string>
#include <vector>


class UDPsocket
{
public:
	typedef struct sockaddr_in sockaddr_in_t;
	typedef struct sockaddr sockaddr_t;
	typedef std::vector<uint8_t> msg_t;

public:
	struct IPv4;

	enum class Status : int
	{
		OK = 0,
		SocketError = -1,
		OpenError = SocketError,
		CloseError = -2,
		ShutdownError = -3,
		BindError = -4,
		ConnectError = BindError,
		SetSockOptError = -5,
		GetSockNameError = -6,
		SendError = -7,
		RecvError = -8,
		//AddressError = -66,
	};

private:
	int sock{ -1 };
	sockaddr_in_t self_addr{};
	socklen_t self_addr_len = sizeof(self_addr);
	sockaddr_in_t peer_addr{};
	socklen_t peer_addr_len = sizeof(peer_addr);

public:
	UDPsocket()
	{
#ifdef _WIN32
		WSAInit();
#endif
		self_addr = IPv4{};
		peer_addr = IPv4{};
	}

	~UDPsocket()
	{
		this->close();
	}

public:
	int open()
	{
		this->close();
		sock = (int)::socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (this->is_closed()) {
			return (int)Status::SocketError;
		}
		return (int)Status::OK;
	}

	int close()
	{
		if (!this->is_closed()) {
#ifdef _WIN32
			int ret = ::shutdown(sock, SD_BOTH);
#else
			int ret = ::shutdown(sock, SHUT_RDWR);
#endif
			if (ret < 0) {
				return (int)Status::ShutdownError;
			}
#ifdef _WIN32
			ret = ::closesocket(sock);
#else
			ret = ::close(sock);
#endif
			if (ret < 0) {
				return (int)Status::CloseError;
			}
			sock = -1;
		}
		return (int)Status::OK;
	}

	bool is_closed() const { return sock < 0; }

public:
	int bind(const IPv4& ipaddr)
	{
		self_addr = ipaddr;
		self_addr_len = sizeof(self_addr);
		int opt = 1;
		int ret = ::setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));
		if (ret < 0) {
			return (int)Status::SetSockOptError;
		}
		ret = ::bind(sock, (sockaddr_t*)&self_addr, self_addr_len);
		if (ret < 0) {
			return (int)Status::BindError;
		}
		ret = ::getsockname(sock, (sockaddr_t*)&self_addr, &self_addr_len);
		if (ret < 0) {
			return (int)Status::GetSockNameError;
		}
		return (int)Status::OK;
	}

	int bind(uint16_t portno)
	{
		auto ipaddr = IPv4::Any(portno);
		return this->bind(ipaddr);
	}

	int bind_any()
	{
		return this->bind(INPORT_ANY);
	}

	int bind_any(uint16_t& portno)
	{
		int ret = this->bind(INPORT_ANY);
		if (ret < 0) {
			return ret;
		}
		portno = IPv4{ self_addr }.port;
		return (int)Status::OK;
	}

public:
	int connect(const IPv4& ipaddr)
	{
		peer_addr = ipaddr;
		peer_addr_len = sizeof(peer_addr);
		int ret = ::connect(sock, (sockaddr_t*)&peer_addr, peer_addr_len);
		if (ret < 0) {
			return (int)Status::ConnectError;
		}
		return (int)Status::OK;
	}

	int connect(uint16_t portno)
	{
		auto ipaddr = IPv4::Loopback(portno);
		return this->connect(ipaddr);
	}

public:
	IPv4 get_self_ip() const
	{
		return self_addr;
	}

	IPv4 get_peer_ip() const
	{
		return peer_addr;
	}

public:
	template <typename T, typename = typename
		std::enable_if<sizeof(typename T::value_type) == sizeof(uint8_t)>::type>
	int send(const T& message, const IPv4& ipaddr) const
	{
		// // UPnP
		// std::string msg = "M-SEARCH * HTTP/1.1\r\nHOST: 239.255.255.250:1900\r\nMAN: ssockp:discover\r\nST: ssockp:all\r\nMX: 1\r\n\r\n";
		sockaddr_in_t addr_in = ipaddr;
		socklen_t addr_in_len = sizeof(addr_in);
		int ret = ::sendto(sock,
			(const char*)message.data(), message.size(), 0,
			(sockaddr_t*)&addr_in, addr_in_len);
		if (ret < 0) {
			return (int)Status::SendError;
		}
		return ret;
	}

	template <typename T, typename = typename
		std::enable_if<sizeof(typename T::value_type) == sizeof(uint8_t)>::type>
	int recv(T& message, IPv4& ipaddr) const
	{
		sockaddr_in_t addr_in;
		socklen_t addr_in_len = sizeof(addr_in);
		typename T::value_type buffer[10 * 1024];
		int ret = ::recvfrom(sock,
			(char*)buffer, sizeof(buffer), 0,
			(sockaddr_t*)&addr_in, &addr_in_len);
		if (ret < 0) {
			return (int)Status::RecvError;
		}
		ipaddr = addr_in;
		message = { std::begin(buffer), std::begin(buffer) + ret };
		return ret;
	}

public:
	int broadcast(int opt) const
	{
		int ret = ::setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (const char*)&opt, sizeof(opt));
		if (ret < 0) {
			return (int)Status::SetSockOptError;
		}
		return (int)Status::OK;
	}

	int interrupt() const
	{
		uint16_t portno = IPv4{ self_addr }.port;
		auto ipaddr = IPv4::Loopback(portno);
		return this->send(msg_t{}, ipaddr);
	}

public:
	struct IPv4
	{
		std::array<uint8_t, 4> octets{};
		uint16_t port{};

	public:
		IPv4()
		{
		}

		IPv4(const std::string& ipaddr, uint16_t portno)
		{
			int ret = ::inet_pton(AF_INET, ipaddr.c_str(), (uint32_t*)octets.data());
			if (ret > 0) {
				port = portno;
			} else {
				//throw std::runtime_error(Status::AddressError)
			}
		}

		IPv4(uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint16_t portno)
		{
			octets[0] = a;
			octets[1] = b;
			octets[2] = c;
			octets[3] = d;
			port = portno;
		}

		IPv4(const sockaddr_in_t& addr_in)
		{
			*(uint32_t*)octets.data() = addr_in.sin_addr.s_addr;
			port = ntohs(addr_in.sin_port);
		}

		operator sockaddr_in_t() const
		{
			sockaddr_in_t addr_in;
			std::memset(&addr_in, 0, sizeof(addr_in));
			addr_in.sin_family = AF_INET;
			addr_in.sin_addr.s_addr = *(uint32_t*)octets.data();
			addr_in.sin_port = htons(port);
			return addr_in;
		}

	private:
		IPv4(uint32_t ipaddr, uint16_t portno)
		{
			*(uint32_t*)octets.data() = htonl(ipaddr);
			port = portno;
		}

	public:
		static IPv4 Any(uint16_t portno) { return IPv4{ INADDR_ANY, portno }; }
		static IPv4 Loopback(uint16_t portno) { return IPv4{ INADDR_LOOPBACK, portno }; }
		static IPv4 Broadcast(uint16_t portno) { return IPv4{ INADDR_BROADCAST, portno }; }

	public:
		const uint8_t& operator[](size_t octet) const { return octets[octet]; }
		uint8_t& operator[](size_t octet) { return octets[octet]; }

	public:
		bool operator==(const IPv4& other) const {
			return this->octets == other.octets && this->port == other.port;
		}

		bool operator!=(const IPv4& other) const {
			return !(*this == other);
		}

	public:
		std::string addr_string() const {
			return std::to_string(octets[0]) +
			 '.' + std::to_string(octets[1]) +
			 '.' + std::to_string(octets[2]) +
			 '.' + std::to_string(octets[3]);
		}

		std::string port_string() const {
			return std::to_string(port);
		}

		std::string to_string() const {
			return this->addr_string() + ':' + this->port_string();
		}

		operator std::string() const { return this->to_string(); }
	};

#ifdef _WIN32
public:
	static WSADATA* WSAInit()
	{
		static WSADATA wsa;
		static struct WSAContext {
			WSAContext(WSADATA* wsa) {
				WSAStartup(0x0202, wsa);
			}
			~WSAContext() {
				WSACleanup();
			}
		} context{ &wsa };
		return &wsa;
	}
#endif
};

namespace std
{
    template<> struct hash<UDPsocket::IPv4>
    {
        typedef UDPsocket::IPv4 argument_type;
        typedef size_t result_type;
        result_type operator()(argument_type const& ipaddr) const noexcept
        {
            result_type const h1{ std::hash<uint32_t>{}(*(uint32_t*)ipaddr.octets.data()) };
            result_type const h2{ std::hash<uint16_t>{}(ipaddr.port) };
            return h1 ^ (h2 << 1);
        }
    };
}

#endif	// TELLO_ONLY_DECLARE

// =========================================
// ===                                   ===
// ===   End of the UDPsocket library    ===
// ===                                   ===
// =========================================






// =========================================
// ===                                   ===
// ===      Begin of Tello library       ===
// ===                                   ===
// =========================================
//
// Tello SDK 2.0:
// https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf
//

#ifndef TELLO_ONLY_DECLARE

#include <thread>
#include <chrono>
#include <mutex>
#include <queue>
#include <functional>

#ifndef _MSC_VER
#define __FUNCTION__ __PRETTY_FUNCTION__
#endif

#endif	// TELLO_ONLY_DECLARE

#define TELLO_DEFAULT_IP "192.168.10.1"
#define TELLO_DEFAULT_COMMAND_PORT 8889
#define TELLO_DEFAULT_DATA_PORT 8890
#define TELLO_DEFAULT_VIDEO_PORT 11111
#define TELLO_DEFAULT_LOCAL_PORT 36085
#define TELLO_DEFAULT_ACTION_TIMEOUT 15000

#define __LOG_COLOR_RED "1;91"
#define __LOG_COLOR_GREEN "0;92"
#define __LOG_COLOR_BLUE "1;94"
#define __LOG_COLOR_YELLOW "0;93"
#define __LOG_COLOR_WHITE "0;97"
#define __LOG_COLOR(color, msg, ...) printf("\033[%sm" ##msg "\033[m\n", color, ##__VA_ARGS__)

#define PRINTF(fmt, ...) __LOG_COLOR(__LOG_COLOR_WHITE, fmt, ##__VA_ARGS__)
#define PRINTF_INFO(fmt, ...) __LOG_COLOR(__LOG_COLOR_GREEN, fmt, ##__VA_ARGS__)
#define PRINTF_WARN(fmt, ...) __LOG_COLOR(__LOG_COLOR_YELLOW, fmt, ##__VA_ARGS__)
#define PRINTF_ERROR(fmt, ...) __LOG_COLOR(__LOG_COLOR_RED, fmt, ##__VA_ARGS__)

#ifdef TELLO_DEBUG
	#define PRINTF_DEBUG(fmt, ...) __LOG_COLOR(__LOG_COLOR_BLUE, fmt, ##__VA_ARGS__)
#else
	#define PRINTF_DEBUG(fmt, ...)
#endif

enum class FlipDirection {
	LEFT = 'l',
	RIGHT = 'r',
	FORWARD = 'f',
	BACK = 'b'
};

enum class MP_DetectDir {
	DOWNWARD_ONLY = 0,
	FORWARD_ONLY = 1,
	BOTH = 2
};

class Tello {
	
	class SyncSocket {
    public:
		SyncSocket(const std::string& targetIP, uint16_t targetPort, uint16_t sourcePort) {
            this->targetIP = targetIP;
            this->targetPort = targetPort;
			if (socket.open() < 0) {
				PRINTF_ERROR("%s(): socket.open() failed.", __FUNCTION__);
				return;
			}
			if (socket.bind(sourcePort) < 0) {
				PRINTF_ERROR("%s(): socket.bind() failed. The port %d may be in use by another application.", __FUNCTION__, sourcePort);
				return;
			}
        }

        bool send(const std::string& data) {
            return socket.send(data, UDPsocket::IPv4::IPv4(targetIP, targetPort)) >= 0;
        }

		std::pair<bool, std::string> recv() {
			std::string data;
			UDPsocket::IPv4 sender;
			int result = socket.recv(data, sender);

			if (result < 0)
				return std::make_pair(false, "");
			
			return std::make_pair(true, data);
		}

    private:
        UDPsocket socket;
        std::string targetIP;
        uint16_t targetPort = 0;
    };

	class AsyncSocket {
	public:
		AsyncSocket(uint16_t port, std::function<void(const std::string&, const UDPsocket::IPv4&)> callback) {
			this->callback = callback;
			if (socket.open() < 0) {
				PRINTF_ERROR("%s(): socket.open() failed.", __FUNCTION__);
				return;
			}
			if (socket.bind(port) < 0) {
				PRINTF_ERROR("%s(): socket.bind() failed. The port %d may be in use by another application.", __FUNCTION__, port);
				return;
			}
			listener = std::thread([&] { listen(); });
		}

		~AsyncSocket() {
			terminate = true;
			if (socket.interrupt() < 0) PRINTF_ERROR("%s(): socket.interrupt() failed. Cannot join thread.", __FUNCTION__);
			listener.join();
		}

		bool send(const std::string& ip, uint16_t port, const std::string& data) {
			return socket.send(data, UDPsocket::IPv4::IPv4(ip, port)) >= 0;
		}

	private:
		void listen() {
			while (!terminate) {
				int error = socket.recv(data, ipaddr);
				if (error < 0) {
					PRINTF_ERROR("%s(): socket.recv() failed: Error code %d", __FUNCTION__, error);
					continue;
				}

				if (callback && !terminate)
					callback(data, ipaddr);     // Data was received
			}
		}

	private:
		UDPsocket socket;
		UDPsocket::IPv4 ipaddr;
		std::string data;

		std::thread listener;
		std::atomic<bool> terminate = false;
		std::function<void(const std::string&, const UDPsocket::IPv4&)> callback;
	};

	class MissionPadAPI {
	public:
		MissionPadAPI(Tello* tello) : tello(tello) {}

		bool enable_pad_detection() { return tello->execute_command("mon"); }
		bool disable_pad_detection() { return tello->execute_command("moff"); }
		bool set_pad_detection_direction(enum class MP_DetectDir direction) {
			return tello->execute_command("mdirection", static_cast<int>(direction));
		}
		
		bool fly_straight_to_pad(float x, float y, float z, float speed, int mp_id) {
			return tello->execute_command("go", x, y, z, speed, mp_id);
		}
		bool fly_arc_to_pad(float start_x, float start_y, float start_z, float end_x, float end_y, float end_z, float speed_cmps, int mp_id) {
			return tello->execute_command("curve", start_x, start_y, start_z, end_x, end_y, end_z, speed_cmps, mp_id);
		}
		bool jump_to_next_pad(float x, float y, float z, float speed, float yaw, int mp_id1, int mp_id2) {
			return tello->execute_command("jump", x, y, z, speed, yaw, mp_id1, mp_id2);
		}
	
	private:
		Tello* tello;
	};

public:
	Tello(int timeout = TELLO_DEFAULT_ACTION_TIMEOUT,
		uint16_t commandPort = TELLO_DEFAULT_COMMAND_PORT,
		uint16_t dataPort = TELLO_DEFAULT_DATA_PORT,
		uint16_t videoPort = TELLO_DEFAULT_VIDEO_PORT,
		uint16_t localPort = TELLO_DEFAULT_LOCAL_PORT) :
		commandServer(localPort, [&](auto& data, auto& ip) { OnResponse(data); }),
		dataServer(dataPort, [&](auto& data, auto& ip) { OnDataStream(data); }),
		videoServer(videoPort, [&](auto& data, auto& ip) { OnVideoStream(data); }),
		commandPort(commandPort),
		timeout(timeout),
		missionPadAPI(this)
    {
	}

	~Tello() {
		if (connected) {
			execute_command("land", true);
			execute_command("streamoff", true);
		}
	}

	bool connect(const std::string& ipAddress = TELLO_DEFAULT_IP) {
		this->ipAddress = ipAddress;
		connected = true;

		PRINTF_INFO("[Tello] Connecting to %s", ipAddress.c_str());
		if (!execute_command("command")) {
			PRINTF_ERROR("[Tello] Tello not found. Please check the connection");
			connected = false;
			return false;
		}
		float battery = get_battery_level();
		PRINTF_INFO("[Tello] Connected: Battery level %.0f%%", battery);
		if (battery < 5.f) {
			PRINTF_ERROR("[Tello] ERROR: The battery level is below 5%%! Do not fly anymore!");
			return false;
		}
		else if (battery < 10.f) {
			PRINTF_WARN("[Tello] WARNING: The battery level is below 10%%!");
		}
		return true;
	}





	

	// =============================================
	// ===                                       ===
	// ===  Implementation of the Tello SDK 2.0  ===
	// ===                                       ===
	// =============================================

	// === Control Commands ===
	
	// 'command' is implemented elsewhere
	bool takeoff() { return execute_command("takeoff"); }
	bool land() { return execute_command("land"); }
	// 'streamon' is implemented elsewhere
	// 'streamoff' is implemented elsewhere
	bool emergency() { return execute_command("emergency"); }
	
	bool move_up(float distance_cm) { return execute_command("up", distance_cm); }
	bool move_down(float distance_cm) { return execute_command("down", distance_cm); }
	bool move_left(float distance_cm) { return execute_command("left", distance_cm); }
	bool move_right(float distance_cm) { return execute_command("right", distance_cm); }
	bool move_forward(float distance_cm) { return execute_command("forward", distance_cm); }
	bool move_back(float distance_cm) { return execute_command("back", distance_cm); }

	bool turn_right(float angle_deg) { return execute_command("cw", angle_deg); }
	bool turn_left(float angle_deg) { return execute_command("ccw", angle_deg); }
	
	bool flip(enum class FlipDirection flipDirection) { 
		return execute_command("flip", static_cast<char>(flipDirection));
	}
		
	bool move_by(float x, float y, float z, float speed_cmps) { return execute_command("go", x, y, z, speed_cmps); }
	bool stop() { return execute_command("stop"); }		// Can be called at any time
	
	bool fly_arc(float start_x, float start_y, float start_z, float end_x, float end_y, float end_z, float speed_cmps) { 
		return execute_command("curve", start_x, start_y, start_z, end_x, end_y, end_z, speed_cmps); 
	}
	

	// === Set Commands ===
	
	bool set_speed(float speed) { return execute_command("speed", speed); }
	bool move(float left_right, float forward_back, float up_down, float yaw) { 
		return execute_command("rc", left_right, forward_back, up_down, yaw); 
	}
	bool set_wifi_password(const std::string& ssid, const std::string& password) {
		return execute_command("wifi", ssid, password);
	}
	bool connect_to_wifi(const std::string& ssid, const std::string& password) {
		return execute_command("ap", ssid, password);
	}


	
	// === Read Commands ===
	
	float get_speed() { return get_float("speed?"); }
	float get_battery_level() { return get_float("battery?"); }
	std::string get_flight_time() { return get_str("time?"); }
	std::string get_wifi_snr() { return get_str("wifi?"); }
	std::string get_sdk_version() { return get_str("sdk?"); }
	std::string get_serial_number() { return get_str("sn?"); }
	
	
	
	// === Mission Pad Commands ===
	MissionPadAPI missionPadAPI;

	
	// This function allows you to send any string directly to the Tello, in case
	// it should ever be necessary. It waits for an 'ok' response and returns false
	// in case of timeout.
	bool execute_manual_command(const std::string& command) {
		execute_command(command);
	}
	
	// This function allows you to send any command directly and return the raw response, 
	// this might be used for reading a sensor value. String must be parsed to whatever datatype
	// you expect to receive
	std::string get_manual_response(const std::string& command) {
		get_reading(command);
	}
	
	// This function tells you if the connection has been established once.
	// It will not ever go to false again
	bool is_connected() {
		return connected;
	}

	// Sleep for a specified number of milliseconds
	void sleep(int ms) {
		std::this_thread::sleep_for(std::chrono::milliseconds(ms));
	}

private:

	float get_float(const std::string& cmd) {
		std::string response = get_reading(cmd);
		if (response.empty())
			return 0.f;
		return std::stof(response);
	}

	std::string get_str(const std::string& cmd) {
		return get_reading(cmd);
	}

	template<typename T>
	inline std::string toString(const T& value) { return std::to_string(value); }
	inline std::string toString(const char* str) { return std::string(str); }
	inline std::string toString(const std::string& str) { return str; }

	template<typename T, typename... TArgs>
	bool execute_command(const std::string& str, T arg, TArgs... args) {
		return execute_command(str + " " + toString(arg), args...);
	}

	std::string get_reading(const std::string& str) {
		if (!connected) {
			PRINTF_ERROR("[Tello] %s(): ERROR -> Tello not connected\n", __FUNCTION__);
			return "";
		}
		
		if (!execute_command(str.c_str()))
			return "";
		return response;
	}
	
	bool execute_command(const std::string& str, bool silent = false) {
		if (!connected) {
			PRINTF_ERROR("[Tello] %s(): ERROR -> Tello not connected\n", __FUNCTION__);
			return false;
		}
		
		response.clear();
		responseOK = false;
		responseError = false;
		
		PRINTF_DEBUG("[Tello] DEBUG: Sending command '%s'", str.c_str());

		std::unique_lock<std::mutex> lock(actionMTX);
		if (!commandServer.send(ipAddress, commandPort, str)) {
			if (!silent) PRINTF_ERROR("[Tello] Failed to send command '%s': Socket error", str.c_str());
			return false;
		}

		int result = waitForResponse(timeout);
		if (result == 1) {
			if (!silent) PRINTF_ERROR("[Tello] Failed to send command '%s': Timeout waiting for response", str.c_str());
			return false;
		}
		else if (result == 2) {
			if (!silent) PRINTF_ERROR("[Tello] Failed to send command '%s': The response was error", str.c_str());
			return false;
		}
		return true;
	}
	
	void OnResponse(const std::string& data) {
		if (data == "ok") {
			responseOK = true;
			return;
		}
		else if (data.substr(0, 5) == "error") {
			responseError = true;
			return;
		}
		
		response = data;	// Remember response string
		responseOK = true;
	}

	void OnDataStream(const std::string& data) {

	}

	void OnVideoStream(const std::string& data) {

	}

	int waitForResponse(int timeout_ms, int interval_us = 100) {
		using namespace std::chrono;
		
		auto start = system_clock::now();
		while (!responseOK) {
			std::this_thread::sleep_for(microseconds(interval_us));
			if (duration_cast<milliseconds>(system_clock::now() - start).count() > timeout_ms) {
				return 1;
			}
			if (responseError) {
				return 2;
			}
		}
		return 0;
	}

private:
	AsyncSocket commandServer;
	AsyncSocket dataServer;
	AsyncSocket videoServer;

	bool connected = false;

	std::string ipAddress;
	uint16_t commandPort = 0;
	std::string response;

	std::mutex actionMTX;
	std::atomic<bool> responseOK = false;
	std::atomic<bool> responseError = false;
	bool inAir = false;
	int timeout = 0;
};





#endif // _TELLO_H
