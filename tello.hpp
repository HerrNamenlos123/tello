
#ifndef _TELLO_H
#define _TELLO_H




// ====================================================================================
// ===                                                                              ===
// ===   Begin of the UDPsocket library (https://github.com/barczynsky/UDPsocket)   ===
// ===                                                                              ===
// ====================================================================================

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

#include <thread>
#include <chrono>
#include <mutex>
#include <queue>
#include <functional>

#ifndef _MSC_VER
#define __FUNCTION__ __PRETTY_FUNCTION__
#endif

#define CHECK_CONNECTION(_return) \
	if (!connected) { \
		fprintf(stderr, "[Tello] %s(): ERROR -> Tello not connected\n", __FUNCTION__); \
		return _return; \
	}

#define TELLO_DEFAULT_IP "192.168.10.1"
#define TELLO_DEFAULT_COMMAND_PORT 8889
#define TELLO_DEFAULT_DATA_PORT 8890
#define TELLO_DEFAULT_VIDEO_PORT 11111
#define TELLO_DEFAULT_LOCAL_PORT 36085
#define TELLO_DEFAULT_ACTION_TIMEOUT 15000

#define __LOG_COLOR_RED "1;31"
#define __LOG_COLOR_GREEN "0;32"
#define __LOG_COLOR_BLUE "1;34"
#define __LOG_COLOR_YELLOW "0;33"
#define __LOG_COLOR_WHITE "0;37"
#define __LOG_COLOR(color, msg, ...) printf("\033[%sm" ##msg "\033[m\n", color, ##__VA_ARGS__)
#define PRINTF(fmt, ...) __LOG_COLOR(__LOG_COLOR_WHITE, fmt, ##__VA_ARGS__)
#define PRINTF_DEBUG(fmt, ...) __LOG_COLOR(__LOG_COLOR_BLUE, fmt, ##__VA_ARGS__)
#define PRINTF_INFO(fmt, ...) __LOG_COLOR(__LOG_COLOR_GREEN, fmt, ##__VA_ARGS__)
#define PRINTF_WARN(fmt, ...) __LOG_COLOR(__LOG_COLOR_YELLOW, fmt, ##__VA_ARGS__)
#define PRINTF_ERROR(fmt, ...) __LOG_COLOR(__LOG_COLOR_RED, fmt, ##__VA_ARGS__)

#define __DEFINE_SIMPLE_ACTION(name, command) \
	bool name() { \
		std::unique_lock<std::mutex> lock(actionMTX); \
		if (!execute_command(command)) \
			return false; \
		return true; \
	}

#define __DEFINE_ACTION_1P(name, command, p1) \
	bool name(int p1) { \
		std::unique_lock<std::mutex> lock(actionMTX); \
		auto str = std::string(command) \
			+ " " + std::to_string(p1); \
		if (!execute_command(str.c_str())) \
			return false; \
		return true; \
	}

#define __DEFINE_ACTION_1P_CHAR(name, command, p1) \
	bool name(char p1) { \
		std::unique_lock<std::mutex> lock(actionMTX); \
		auto str = std::string(command) \
			+ " " + p1; \
		if (!execute_command(str.c_str())) \
			return false; \
		return true; \
	}

#define __DEFINE_ACTION_2P(name, command, p1, p2) \
	bool name(int p1, int p2) { \
		std::unique_lock<std::mutex> lock(actionMTX); \
		auto str = std::string(command) \
			+ " " + std::to_string(p1) \
			+ " " + std::to_string(p2); \
		if (!execute_command(str.c_str())) \
			return false; \
		return true; \
	}

#define __DEFINE_ACTION_4P(name, command, p1, p2, p3, p4) \
	bool name(int p1, int p2, int p3, int p4) { \
		std::unique_lock<std::mutex> lock(actionMTX); \
		auto str = std::string(command) \
			+ " " + std::to_string(p1) \
			+ " " + std::to_string(p2) \
			+ " " + std::to_string(p3) \
			+ " " + std::to_string(p4); \
		if (!execute_command(str.c_str())) \
			return false; \
		return true; \
	}

#define __DEFINE_ACTION_5P(name, command, p1, p2, p3, p4, p5) \
	bool name(int p1, int p2, int p3, int p4, int p5) { \
		std::unique_lock<std::mutex> lock(actionMTX); \
		auto str = std::string(command) \
			+ " " + std::to_string(p1) \
			+ " " + std::to_string(p2) \
			+ " " + std::to_string(p3) \
			+ " " + std::to_string(p4) \
			+ " " + std::to_string(p5); \
		if (!execute_command(str.c_str())) \
			return false; \
		return true; \
	}

#define __DEFINE_ACTION_7P(name, command, p1, p2, p3, p4, p5, p6, p7) \
	bool name(int p1, int p2, int p3, int p4, int p5, int p6, int p7) { \
		std::unique_lock<std::mutex> lock(actionMTX); \
		auto str = std::string(command) \
			+ " " + std::to_string(p1) \
			+ " " + std::to_string(p2) \
			+ " " + std::to_string(p3) \
			+ " " + std::to_string(p4) \
			+ " " + std::to_string(p5) \
			+ " " + std::to_string(p6) \
			+ " " + std::to_string(p7); \
		if (!execute_command(str.c_str())) \
			return false; \
		return true; \
	}

#define __DEFINE_ACTION_8P(name, command, p1, p2, p3, p4, p5, p6, p7, p8) \
	bool name(int p1, int p2, int p3, int p4, int p5, int p6, int p7, int p8) { \
		std::unique_lock<std::mutex> lock(actionMTX); \
		auto str = std::string(command) \
			+ " " + std::to_string(p1) \
			+ " " + std::to_string(p2) \
			+ " " + std::to_string(p3) \
			+ " " + std::to_string(p4) \
			+ " " + std::to_string(p5) \
			+ " " + std::to_string(p6) \
			+ " " + std::to_string(p7) \
			+ " " + std::to_string(p8); \
		if (!execute_command(str.c_str())) \
			return false; \
		return true; \
	}

enum FlipDirection {
	LEFT,
	RIGHT,
	FORWARD,
	BACK
};

class Tello {
public:
	
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
		timeout(timeout)
    {
	}

	~Tello() {
		std::unique_lock<std::mutex> lock(actionMTX);

		if (connected)
			execute_command("land", true);
	}

	bool connect(const std::string& ipAddress = TELLO_DEFAULT_IP) {
		this->ipAddress = ipAddress;
		connected = true;

		PRINTF_INFO("[Tello] Connecting to %s", ipAddress.c_str());
		{
			std::unique_lock<std::mutex> lock(actionMTX);
			if (!send_command_raw("command")) {
				PRINTF_ERROR("[Tello] Tello not found. Please check the connection");
				connected = false;
				return false;
			}
		}
		float battery = get_battery_level();
		PRINTF_INFO("[Tello] Connected: Battery level %.0f%%", battery);
		if (battery < 5) {
			PRINTF_ERROR("[Tello] ERROR: The battery level is below 5%%! Do not fly anymore!");
			return false;
		}
		else if (battery < 10) {
			PRINTF_WARN("[Tello] WARNING: The battery level is below 10%%!");
		}
		return true;
	}





	
	// 'command' is implemented elsewhere
	__DEFINE_SIMPLE_ACTION(takeoff, "takeoff");
	__DEFINE_SIMPLE_ACTION(land, "land");
	// 'streamon' is implemented elsewhere
	// 'streamoff' is implemented elsewhere
	__DEFINE_SIMPLE_ACTION(emergency, "emergency");
	
	__DEFINE_ACTION_1P(move_up, "up", distance_cm);
	__DEFINE_ACTION_1P(move_down, "down", distance_cm);
	__DEFINE_ACTION_1P(move_left, "left", distance_cm);
	__DEFINE_ACTION_1P(move_right, "right", distance_cm);
	__DEFINE_ACTION_1P(move_forward, "forward", distance_cm);
	__DEFINE_ACTION_1P(move_back, "back", distance_cm);
	
	__DEFINE_ACTION_1P(turn_right, "cw", angle_deg);
	__DEFINE_ACTION_1P(turn_left, "ccw", angle_deg);
	
	bool flip(enum FlipDirection flipDirection) {
		char c = 0;
		if (flipDirection == FlipDirection::FORWARD) c = 'f';
		if (flipDirection == FlipDirection::BACK) c = 'b';
		if (flipDirection == FlipDirection::LEFT) c = 'l';
		if (flipDirection == FlipDirection::RIGHT) c = 'r';
		return __flip(c);
	}
	private: __DEFINE_ACTION_1P_CHAR(__flip, "flip", flipdir);
	public:
		
	__DEFINE_ACTION_4P(move_by, "go", x, y, z, speed);
	__DEFINE_SIMPLE_ACTION(stop, "stop");		// Can be called at any time
	
	__DEFINE_ACTION_7P(fly_arc, "curve", start_x, start_y, start_z, end_x, end_y, end_z, speed_cmps);
	__DEFINE_ACTION_5P(fly_to_mission_pad, "go", x, y, z, speed, mp_id);
	__DEFINE_ACTION_8P(fly_arc_to_mission_pad, "curve", start_x, start_y, start_z, end_x, end_y, end_z, speed, mp_id);
	__DEFINE_ACTION_7P(fly_and_look_to_next_mission_pad, "jump", x, y, z, speed, yaw, mp_id1, mp_id2);






	


	
	float get_battery_level() {
		std::unique_lock<std::mutex> lock(actionMTX);

		std::string response = read_command("battery?");
		if (response.empty())
			return 0.f;
		return std::stof(response);
	}

	// This function tells you if the connection has been established once.
	// It will not ever go to false again
	bool is_connected() {
		return connected;
	}

private:

	// Internal: Call a command and return the response string. 
	// WARNING: THIS FUNCTION IS NOT THREAD-SAFE!!!
	std::string read_command(const char* str) {
		CHECK_CONNECTION("");
		if (!send_command_raw(str))
			return "";
		return response;
	}

	// Internal: Call a command and wait until 'ok' is received.
	// WARNING: THIS FUNCTION IS NOT THREAD-SAFE!!!
	bool execute_command(const char* str, bool silent = false) {
		CHECK_CONNECTION(false);
		return send_command_raw(str, silent);
	}
	
	bool send_command_raw(const char* str, bool silent = false) {
#ifdef TELLO_DEBUG
		printf("[DEBUG] Sending command '%s'\n", str);
#endif
		
		if (!commandServer.send(ipAddress, commandPort, str)) {
			if (!silent) PRINTF_ERROR("[Tello] Failed to send command '%s': Socket error", str);
			return false;
		}

		int result = waitForResponse(timeout);
		if (result == 1) {
			if (!silent) PRINTF_ERROR("[Tello] Failed to send command '%s': Timeout waiting for response", str);
			return false;
		}
		else if (result == 2) {
			if (!silent) PRINTF_ERROR("[Tello] Failed to send command '%s': The response was error", str);
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
		
		response = data;
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
				responseOK = false;
				responseError = false;
				return 1;
			}
			if (responseError) {
				responseOK = false;
				responseError = false;
				return 2;
			}
		}
		responseOK = false;
		responseError = false;
		return 0;
	}

	void sleep(int ms) {
		std::this_thread::sleep_for(std::chrono::milliseconds(ms));
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
