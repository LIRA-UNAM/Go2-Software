#!/usr/bin/env python3
"""
Publica imágenes de la cámara Go2 vía GStreamer (multicast H264).
Usa PyGObject/Gst porque OpenCV suele venir sin soporte GStreamer.
Requiere: python3-gi, gir1.2-gstreamer-1.0 (o python3-gst-1.0)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

try:
    import gi
    gi.require_version('Gst', '1.0')
    gi.require_version('GstApp', '1.0')
    from gi.repository import Gst, GstApp
    HAS_GST = True
except (ImportError, ValueError) as e:
    HAS_GST = False
    _gst_error = str(e)


class GStreamerImagePublisher(Node):
    def __init__(self):
        super().__init__('gstreamer_image_publisher')

        if not HAS_GST:
            raise RuntimeError(
                f"No se pudo cargar GStreamer (gi.repository.Gst): {_gst_error}. "
                "Instala: sudo apt install python3-gst-1.0 gir1.2-gstreamer-1.0"
            )

        # Parámetros
        self.declare_parameter("gstreamer_pipeline", "")
        self.declare_parameter("multicast_address", "230.1.1.1")
        self.declare_parameter("port", 1720)
        self.declare_parameter("multicast_iface", "eth0")
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("topic_name", "/camera/image_raw")
        self.declare_parameter("frame_id", "camera_link")

        gstreamer_pipeline = self.get_parameter("gstreamer_pipeline").value
        if gstreamer_pipeline:
            gstreamer_str = gstreamer_pipeline
        else:
            multicast_address = self.get_parameter("multicast_address").value
            port = self.get_parameter("port").value
            multicast_iface = self.get_parameter("multicast_iface").value
            width = self.get_parameter("width").value
            height = self.get_parameter("height").value
            gstreamer_str = (
                f"udpsrc address={multicast_address} port={port} multicast-iface={multicast_iface} "
                "! application/x-rtp, media=video, encoding-name=H264 "
                "! rtph264depay ! h264parse ! avdec_h264 "
                "! videoconvert "
                f"! video/x-raw,width={width},height={height},format=BGR ! appsink name=mysink drop=1 emit-signals=true"
            )

        self.topic_name = self.get_parameter("topic_name").value
        self.frame_id = self.get_parameter("frame_id").value
        self.width = self.get_parameter("width").value
        self.height = self.get_parameter("height").value

        # Pipeline GStreamer
        Gst.init(None)
        self.pipeline = Gst.parse_launch(gstreamer_str)
        self.appsink = self.pipeline.get_by_name("mysink")
        if self.appsink is None:
            for elem in self.pipeline.iterate_elements():
                if elem.get_factory().get_name() == "appsink":
                    self.appsink = elem
                    break
        if self.appsink is None:
            raise RuntimeError("No se encontró appsink en el pipeline")
        self.appsink.set_property("emit-signals", True)
        self.appsink.set_property("drop", True)
        self.appsink.set_property("max-buffers", 1)

        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("No se pudo abrir el stream GStreamer")

        self.publisher_ = self.create_publisher(Image, self.topic_name, 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 FPS

        self.get_logger().info(f"Publicando imágenes en {self.topic_name}")

    def timer_callback(self):
        sample = self.appsink.try_pull_sample(int(0.1 * Gst.SECOND))
        if sample is None:
            return
        buffer = sample.get_buffer()
        if buffer is None:
            return
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            return
        try:
            frame = np.ndarray(
                shape=(self.height, self.width, 3),
                dtype=np.uint8,
                buffer=map_info.data
            ).copy()
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.publisher_.publish(msg)
        finally:
            buffer.unmap(map_info)

    def destroy_node(self):
        if hasattr(self, "pipeline") and self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GStreamerImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
