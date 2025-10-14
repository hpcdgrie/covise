#ifndef COVISE_MQTT_CLIENT_H
#define COVISE_MQTT_CLIENT_H

#include <DataClient/DataClient.h>

#include <mqtt/async_client.h>
#include <mqtt/message.h>
#include <mutex>
#include <deque>
#include <map>
#include <sstream>
#include <nlohmann/json.hpp>

namespace opencover { namespace dataclient {

struct TopicBuffer {
    std::deque<MultiDimensionalArray<double>> values;
    size_t maxQueue = 10;
};

class MqttImpl : public Client, public virtual mqtt::callback {
public:
    MqttImpl(const std::string &uri, const std::string &clientId);

    void connect() override;
    void disconnect() override ;
    bool isConnected() const override;

    std::vector<std::string> allAvailableScalars() const;
    std::vector<std::string> allAvailableArrays() const override;
    std::unique_ptr<ObserverHandle> observeNode(const std::string &name) override;

    double getNumericScalar(const std::string &name) override;

    size_t numNodeUpdates(const std::string &name) override;

    template<typename T>
    MultiDimensionalArray<T> getArrayImpl(const std::string &name)
    {
        std::lock_guard<std::mutex> g(m_mutex);
        MultiDimensionalArray<T> out;
        auto it = m_buffers.find(name);
        if(it==m_buffers.end() || it->second.values.empty()) return out;
        auto val = it->second.values.front();
        it->second.values.pop_front();
        out.dimensions = val.dimensions;
        out.timestamp = val.timestamp;
        out.data.assign(val.data.begin(), val.data.end());
        return out;
    }

    // mqtt::callback
    void message_arrived(mqtt::const_message_ptr msg) override;

private:
    mqtt::async_client m_client;
    mutable std::mutex m_mutex;
    std::map<std::string, TopicBuffer> m_buffers;
};

std::unique_ptr<Client> makeMqttClient(const std::string &uri, const std::string &clientId);

}} // namespace

#endif // COVISE_MQTT_CLIENT_H
