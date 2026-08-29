#include <iostream>
#include <string>
#include <memory>
#include <csignal>
#include <cstring>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

// Global objects
GstElement *pipeline = nullptr;
GMainLoop *loop = nullptr;
GstElement *appsrc = nullptr;
bool configured = false;

// Command line args
std::string TARGET_IP = "127.0.0.1";
int TARGET_PORT = 5600;
int FRAMERATE = 10;

// Image callback
void on_frame(const gz::msgs::Image &msg) {
    if (!appsrc) return;

    // Configure Caps on the first frame
    if (!configured) {
        GstCaps *caps = gst_caps_new_simple("video/x-raw",
            "format", G_TYPE_STRING, "RGB",
            "width", G_TYPE_INT, msg.width(),
            "height", G_TYPE_INT, msg.height(),
            "framerate", GST_TYPE_FRACTION, FRAMERATE, 1,
            NULL);
        
        gst_app_src_set_caps(GST_APP_SRC(appsrc), caps);
        gst_caps_unref(caps);
        
        std::cout << "Configured GStreamer for " << msg.width() << "x" << msg.height() << std::endl;
        configured = true;
    }

    // Copy to GStreamer Buffer
    gsize size = msg.data().size();
    GstBuffer *buffer = gst_buffer_new_allocate(NULL, size, NULL);

    if (buffer) {
        // Copy the pixel data from Gazebo message to GStreamer buffer
        gst_buffer_fill(buffer, 0, msg.data().c_str(), size);

        // Push to pipeline
        GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer);
        if (ret != GST_FLOW_OK) {
            // Use GLib logging for thread safety
            g_warning("Error pushing buffer to appsrc");
        }
    } else {
        g_warning("Failed to allocate GStreamer buffer");
    }
}

bool check_nvidia_encoder() {
    GstElementFactory *factory = gst_element_factory_find("nvcudah264enc");
    if (factory) {
        gst_object_unref(factory);
        return true;
    }
    return false;
}

void sigint_handler(int sig) {
    (void)sig; // Suppress unused warning
    if (loop) g_main_loop_quit(loop);
}

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cerr << "Usage: ./gz_gst_bridge <gz_topic> <ip> [port] [framerate]\n";
        return 1;
    }
    std::string topic = argv[1];
    TARGET_IP = argv[2];
    if (argc > 3) TARGET_PORT = std::stoi(argv[3]);
    if (argc > 4) FRAMERATE = std::stoi(argv[4]);

    // Initialize GStreamer and Gazebo Transport
    gst_init(&argc, &argv);    
    gz::transport::Node node;

    // Build Pipeline String
    std::string pipeline_str;
    std::string ip_port = "host=" + TARGET_IP + " port=" + std::to_string(TARGET_PORT);
    if (check_nvidia_encoder()) {
        std::cout << "Using NVIDIA GPU Encoder (nvh264enc)\n";
        pipeline_str = "appsrc name=gz_source ! queue max-size-buffers=1 leaky=downstream ! "
                        "videoconvert ! "
                        "nvcudah264enc preset=p3 tune=low-latency rate-control=cbr bitrate=2048 gop-size=30 ! "
                        "rtph264pay config-interval=1 mtu=1400 ! udpsink sync=false " + ip_port;
    } else {
        std::cout << "Using CPU Encoder (x264enc)\n";
        pipeline_str = "appsrc name=gz_source ! queue max-size-buffers=1 leaky=downstream ! "
                        "videoconvert ! "
                        "x264enc speed-preset=ultrafast tune=zerolatency bitrate=2048 key-int-max=30 ! "
                        "rtph264pay config-interval=1 mtu=1400 ! udpsink sync=false " + ip_port;
    }
    // Note: re-send full frame every 30 frames

    // Launch Pipeline
    GError *error = nullptr;
    pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
    if (!pipeline) {
        std::cerr << "Failed to create pipeline: " << error->message << std::endl;
        return -1;
    }

    appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "gz_source");
    g_object_set(appsrc, "format", GST_FORMAT_TIME, NULL);
    g_object_set(appsrc, "is-live", TRUE, NULL);
    g_object_set(appsrc, "do-timestamp", TRUE, NULL);
    g_object_set(appsrc, "leaky-type", 2, NULL); // Drop old frames
    g_object_set(appsrc, "max-bytes", 0, NULL);
    g_object_set(appsrc, "max-buffers", 2, NULL); 
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    if (!node.Subscribe(topic, on_frame)) {
        std::cerr << "Error subscribing to " << topic << std::endl;
        return -1;
    }
    
    std::cout << "Streaming " << topic << " to " << TARGET_IP << ":" << TARGET_PORT << " at " << FRAMERATE << " FPS." << std::endl;

    loop = g_main_loop_new(NULL, FALSE);
    signal(SIGINT, sigint_handler);
    g_main_loop_run(loop);

    // Cleanup
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(appsrc);
    gst_object_unref(pipeline);
    g_main_loop_unref(loop);

    return 0;
}
