#include "MqttClient.h"

#include <mqtt/async_client.h>
#include <mutex>
#include <deque>
#include <map>
#include <sstream>
#include <nlohmann/json.hpp>
#include <chrono>

using namespace opencover::dataclient;

const mqtt::string mqtt::message::EMPTY_STR;

// A const binary to use for references
const mqtt::binary mqtt::message::EMPTY_BIN;

MqttImpl::MqttImpl(const std::string &uri, const std::string &clientId)
: m_client(uri, clientId)
{
    m_client.set_callback(*this);
}

void MqttImpl::connect() {
    if(isConnected()) return;
    mqtt::connect_options opts;
    m_client.connect(opts)->wait();
}

void MqttImpl::disconnect()  {
    if(isConnected()) m_client.disconnect()->wait();
}
bool MqttImpl::isConnected() const  { return m_client.is_connected(); }

std::vector<std::string> MqttImpl::allAvailableScalars() const  {
    // MQTT has no discovery; return subscribed topics
    std::lock_guard<std::mutex> g(m_mutex);
    std::vector<std::string> names; names.reserve(m_buffers.size());
    for(auto &kv : m_buffers) names.push_back(kv.first);
    return names;
}
std::vector<std::string> MqttImpl::allAvailableArrays() const  { return allAvailableScalars(); }

std::unique_ptr<ObserverHandle> MqttImpl::observeNode(const std::string &name)  {
    std::lock_guard<std::mutex> g(m_mutex);
    if(!isConnected()) connect();
    if(m_buffers.find(name)==m_buffers.end()) m_buffers[name] = TopicBuffer{};
    m_client.subscribe(name, 0)->wait();
    // Return a trivial handle that on destruction unsubscribes (optional)
    struct H : ObserverHandle { mqtt::async_client *c; std::string t; ~H(){ if(c && c->is_connected()) c->unsubscribe(t)->wait(); } };
    auto h = std::make_unique<H>(); h->c = &m_client; h->t = name; return h;
}

double MqttImpl::getNumericScalar(const std::string &name)  {
    auto arr = getArrayImpl<double>(name);
    if(arr.isScalar()) return arr.data.front();
    return 0.0;
}

size_t MqttImpl::numNodeUpdates(const std::string &name)  {
    std::lock_guard<std::mutex> g(m_mutex);
    auto it = m_buffers.find(name);
    if(it==m_buffers.end()) return 0;
    return it->second.values.size();
}

// mqtt::callback
void MqttImpl::message_arrived(mqtt::const_message_ptr msg)  {
    auto topic = msg->get_topic();
    auto payload = msg->to_string();
    MultiDimensionalArray<double> arr;
    // Try JSON first: { "dimensions":[N,...], "data":[...] }
    try {
        auto j = nlohmann::json::parse(payload);
        if(j.contains("data")){
            arr.data = j.at("data").get<std::vector<double>>();
            if(j.contains("dimensions")) arr.dimensions = j.at("dimensions").get<std::vector<size_t>>();
            if(arr.dimensions.empty()) arr.dimensions = { arr.data.size() };
        }
    } catch(...) {
        // Fallback: CSV list of doubles, interpreted as 1D array
        std::stringstream ss(payload);
        double v; char c;
        while(ss >> v){ arr.data.push_back(v); ss >> c; }
        if(!arr.data.empty()) arr.dimensions = { arr.data.size() };
    }
    if(arr.dimensions.empty()) return;
    arr.timestamp = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());

    std::lock_guard<std::mutex> g(m_mutex);
    auto &buf = m_buffers[topic];
    buf.values.push_back(std::move(arr));
    while(buf.values.size() > buf.maxQueue) buf.values.pop_front();
}

std::unique_ptr<Client> opencover::dataclient::makeMqttClient(const std::string &uri, const std::string &clientId)
{
    return std::unique_ptr<Client>(new MqttImpl(uri, clientId));
}
