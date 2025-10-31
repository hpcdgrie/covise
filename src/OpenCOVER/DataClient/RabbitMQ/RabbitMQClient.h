#ifndef OPENCOVER_RABBITMQCLIENT_H
#define OPENCOVER_RABBITMQCLIENT_H

#include "export.h"

#include <DataClient/DataClient.h>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <typeindex>
#include <vector>

namespace opencover { namespace dataclient {

// Minimal message-backed node state
struct RabbitNode {
    std::string name;                    // queue or routing key
    std::deque<double> values{0.0};      // buffered numeric values
    double timestamp = 0.0;              // last update timestamp (seconds)
};

class RABBITMQEXPORT RabbitMQClient : public Client {
public:
    struct ConnectionParams {
        std::string host = "localhost";
        int port = 5672;        // TCP port
        std::string vhost = "/";
        std::string user = "guest";
        std::string password = "guest";
        std::string exchange;   // optional, empty = direct-to-queue
    };

    explicit RabbitMQClient(const ConnectionParams &params = {});
    ~RabbitMQClient() override;

    // Connection lifecycle
    void connect() override;
    void disconnect() override;
    bool isConnected() const override;

    // Observe a node (consumes from queue named by 'name')
    [[nodiscard]] ObserverHandle observeNode(const std::string &name) override;

    // Pull-style
    double getNumericScalar(const std::string &name, double *timestep = nullptr) override;
    double getNumericScalar(const ObserverHandle &handle, double *timestep = nullptr) override;

    size_t numNodeUpdates(const std::string &name) override;

private:
    std::unique_ptr<detail::MultiDimensionalArrayBase> getArrayImpl(std::type_index type, const std::string &name) override;
    std::vector<std::string> getNodesWith(std::type_index type, bool isScalar) const override;
    std::vector<std::string> getNodesWith(bool isArithmetic, bool isScalar) const override;

    // internals
    void consumerLoop();
    void ensureConsumerFor(const std::string &name, size_t id);

    mutable std::mutex m_mtx;
    std::atomic<bool> m_stop{false};
    std::thread m_thread;

    ConnectionParams m_params;

    // node management
    size_t m_nextId = 1;
    std::map<size_t, std::string> m_idToName;                    // id -> node name
    std::map<std::string, std::unique_ptr<RabbitNode>> m_nodes;  // name -> node

#ifdef HAVE_RABBITMQ
    // rabbitmq-c state
    struct Impl;
    std::unique_ptr<Impl> m_impl;
#endif

    bool m_connected = false;
};

}} // namespace opencover::dataclient

#endif // OPENCOVER_RABBITMQCLIENT_H
