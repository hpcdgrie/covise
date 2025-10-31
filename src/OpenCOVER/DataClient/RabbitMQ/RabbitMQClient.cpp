#include "RabbitMQClient.h"

#include <iostream>
#include <chrono>

#ifdef HAVE_RABBITMQ
// rabbitmq-c headers
#include <amqp.h>
#include <amqp_framing.h>
#include <amqp_tcp_socket.h>

// Internal implementation storage
struct opencover::dataclient::RabbitMQClient::Impl {
    amqp_connection_state_t conn = nullptr;
    int channel = 1;
    std::map<std::string, std::string> nameToConsumerTag; // node name -> consumer tag
};
#endif

using namespace opencover::dataclient;

RabbitMQClient::RabbitMQClient(const ConnectionParams &params)
: m_params(params)
{
}

RabbitMQClient::~RabbitMQClient()
{
    try { disconnect(); } catch(...) {}
}

void RabbitMQClient::connect()
{
    std::lock_guard<std::mutex> lock(m_mtx);
    if (m_connected)
        return;

#ifdef HAVE_RABBITMQ
    m_impl = std::make_unique<Impl>();
#else
    // No-op when RabbitMQ support is disabled
#endif

    m_stop = false;
    m_thread = std::thread(&RabbitMQClient::consumerLoop, this);
    m_connected = true;
    statusChanged();
}

void RabbitMQClient::disconnect()
{
    {
        std::lock_guard<std::mutex> lock(m_mtx);
        if (!m_connected)
            return;
        m_connected = false;
        m_stop = true;
    }

    if (m_thread.joinable())
        m_thread.join();

#ifdef HAVE_RABBITMQ
    if (m_impl && m_impl->conn) {
        // Try to close channel and connection
        amqp_channel_close(m_impl->conn, m_impl->channel, AMQP_REPLY_SUCCESS);
        amqp_connection_close(m_impl->conn, AMQP_REPLY_SUCCESS);
        amqp_destroy_connection(m_impl->conn);
    }
    m_impl.reset();
#endif

    statusChanged();
}

bool RabbitMQClient::isConnected() const
{
    return m_connected;
}

ObserverHandle RabbitMQClient::observeNode(const std::string &name)
{
    std::lock_guard<std::mutex> lock(m_mtx);

    // create node on demand
    auto &nodePtr = m_nodes[name];
    if (!nodePtr)
        nodePtr = std::make_unique<RabbitNode>(RabbitNode{name});

    const size_t id = m_nextId++;
    m_idToName[id] = name;

    if (m_connected) {
        ensureConsumerFor(name, id);
    }

    return ObserverHandle(id, this);
}

void RabbitMQClient::ensureConsumerFor(const std::string &name, size_t /*id*/)
{
#ifdef HAVE_RABBITMQ
    if (!m_impl)
        return;

    if (!m_impl->conn) {
        // establish connection on first need
        m_impl->conn = amqp_new_connection();
        amqp_socket_t *sock = amqp_tcp_socket_new(m_impl->conn);
        if (!sock) {
            std::cerr << "RabbitMQClient: failed to create TCP socket" << std::endl;
            return;
        }
        if (amqp_socket_open(sock, m_params.host.c_str(), m_params.port)) {
            std::cerr << "RabbitMQClient: socket open failed" << std::endl;
            return;
        }

        amqp_rpc_reply_t r = amqp_login(m_impl->conn,
                                        m_params.vhost.c_str(),
                                        0,
                                        131072, // frame max
                                        60,
                                        AMQP_SASL_METHOD_PLAIN,
                                        m_params.user.c_str(),
                                        m_params.password.c_str());
        if (r.reply_type != AMQP_RESPONSE_NORMAL) {
            std::cerr << "RabbitMQClient: login failed" << std::endl;
            return;
        }
        amqp_channel_open(m_impl->conn, m_impl->channel);
        r = amqp_get_rpc_reply(m_impl->conn);
        if (r.reply_type != AMQP_RESPONSE_NORMAL) {
            std::cerr << "RabbitMQClient: channel open failed" << std::endl;
            return;
        }
    }

    // Declare queue and consume
    amqp_bytes_t qname = amqp_cstring_bytes(name.c_str());
    amqp_queue_declare(m_impl->conn, m_impl->channel, qname,
                       0,    // passive
                       0,    // durable
                       0,    // exclusive
                       1,    // auto-delete
                       amqp_empty_table);
    amqp_rpc_reply_t r = amqp_get_rpc_reply(m_impl->conn);
    if (r.reply_type != AMQP_RESPONSE_NORMAL) {
        std::cerr << "RabbitMQClient: queue declare failed for '" << name << "'" << std::endl;
        return;
    }

    std::string ctag = "covise-client-" + name;
    amqp_basic_consume(m_impl->conn, m_impl->channel, qname,
                       amqp_cstring_bytes(ctag.c_str()),
                       1,   // no_local
                       1,   // no_ack
                       0,   // exclusive
                       amqp_empty_table);
    r = amqp_get_rpc_reply(m_impl->conn);
    if (r.reply_type != AMQP_RESPONSE_NORMAL) {
        std::cerr << "RabbitMQClient: basic.consume failed for '" << name << "'" << std::endl;
        return;
    }

    m_impl->nameToConsumerTag[name] = ctag;
#else
    (void)name;
#endif
}

static double now_seconds()
{
    using clock = std::chrono::steady_clock;
    static const auto t0 = clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(clock::now() - t0);
    return dt.count();
}

void RabbitMQClient::consumerLoop()
{
#ifdef HAVE_RABBITMQ
    while (!m_stop.load()) {
        if (!m_impl || !m_impl->conn) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        amqp_envelope_t env;
        amqp_maybe_release_buffers(m_impl->conn);
        amqp_rpc_reply_t ret = amqp_consume_message(m_impl->conn, &env, nullptr, 0);
        if (ret.reply_type == AMQP_RESPONSE_NORMAL) {
            // Determine node by consumer tag
            std::string tag((const char*)env.consumer_tag.bytes, env.consumer_tag.len);

            std::string name;
            {
                std::lock_guard<std::mutex> lock(m_mtx);
                // reverse lookup tag->name
                for (const auto &kv : m_impl->nameToConsumerTag) {
                    if (kv.second == tag) { name = kv.first; break; }
                }
            }

            if (!name.empty()) {
                double value = 0.0;
                if (env.message.body.len > 0) {
                    try {
                        std::string s((const char*)env.message.body.bytes, env.message.body.len);
                        value = std::stod(s);
                    } catch (...) {
                        // ignore parsing error, keep 0.0
                    }
                }

                std::lock_guard<std::mutex> lock(m_mtx);
                auto it = m_nodes.find(name);
                if (it != m_nodes.end() && it->second) {
                    it->second->values.push_back(value);
                    it->second->timestamp = now_seconds();
                }
            }

            amqp_destroy_envelope(&env);
        } else {
            // avoid busy loop on timeout or connection issues
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
#else
    // Stub: no RabbitMQ -> idle loop to allow clean stop
    while (!m_stop.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
#endif
}

// Pull API

double RabbitMQClient::getNumericScalar(const std::string &name, double *timestep)
{
    std::lock_guard<std::mutex> lock(m_mtx);
    auto it = m_nodes.find(name);
    if (it == m_nodes.end() || !it->second || it->second->values.empty()) {
        if (timestep) *timestep = 0.0;
        return 0.0;
    }
    auto &node = *it->second;
    double v = node.values.front();
    if (node.values.size() > 1)
        node.values.pop_front();
    if (timestep) *timestep = node.timestamp;
    return v;
}

double RabbitMQClient::getNumericScalar(const ObserverHandle &handle, double *timestep)
{
    std::string name;
    {
        std::lock_guard<std::mutex> lock(m_mtx);
        for (const auto &kv : m_idToName) {
            if (handle == kv.first) { name = kv.second; break; }
        }
    }
    if (name.empty()) {
        if (timestep) *timestep = 0.0;
        return 0.0;
    }
    return getNumericScalar(name, timestep);
}

size_t RabbitMQClient::numNodeUpdates(const std::string &name)
{
    std::lock_guard<std::mutex> lock(m_mtx);
    auto it = m_nodes.find(name);
    if (it == m_nodes.end() || !it->second)
        return 0;
    return it->second->values.size();
}

std::unique_ptr<detail::MultiDimensionalArrayBase>
RabbitMQClient::getArrayImpl(std::type_index type, const std::string &name)
{
    if (type != std::type_index(typeid(double)))
        return nullptr;
    auto arr = std::make_unique<dataclient::MultiDimensionalArray<double>>();
    arr->dimensions = {1};
    arr->timestamp = 0.0;
    double ts = 0.0;
    arr->data = { getNumericScalar(name, &ts) };
    arr->timestamp = ts;
    return arr;
}

std::vector<std::string> RabbitMQClient::getNodesWith(std::type_index type, bool isScalar) const
{
    std::vector<std::string> out;
    if (type != std::type_index(typeid(double)) || !isScalar)
        return out;
    std::lock_guard<std::mutex> lock(m_mtx);
    out.reserve(m_nodes.size());
    for (auto &kv : m_nodes) out.push_back(kv.first);
    return out;
}

std::vector<std::string> RabbitMQClient::getNodesWith(bool isArithmetic, bool isScalar) const
{
    std::vector<std::string> out;
    if (!isArithmetic || !isScalar)
        return out;
    std::lock_guard<std::mutex> lock(m_mtx);
    out.reserve(m_nodes.size());
    for (auto &kv : m_nodes) out.push_back(kv.first);
    return out;
}
