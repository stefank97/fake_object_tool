#include <rviz_common/tool.hpp>                     // eigenes Tool
#include <rviz_common/display_context.hpp>          // Zugriff auf Node, fixed frame, TF
#include <rviz_common/viewport_mouse_event.hpp>     // Maus-Klicks auswerten
#include <rviz_common/properties/float_property.hpp>// Tool-Properties (width/height/yaw)
#include <rviz_common/properties/string_property.hpp>// z.B. target frame name
#include <rclcpp/rclcpp.hpp>                        // Publisher, Node-Handle
#include <geometry_msgs/msg/point_stamped.hpp>      // Klickpunkt (wie PublishPoint)
#include <geometry_msgs/msg/polygon_stamped.hpp>    // Rechteck als Polygon fürs Filter
#include <cmath>                                    // cos/sin


namespace fake_object_tool {



class RectangleTool : public rviz_common::Tool
{
    Q_OBJECT
public:
    RectangleTool();
    ~RectangleTool() override = default;
    void onInitialize() override;
    void activate() override;
    void deactivate() override;

    int processMouseEvent(rviz_common::ViewportMouseEvent& event ) override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_;

    geometry_msgs::msg::PolygonStamped buildRectangle(const geometry_msgs::msg::Point& c,
                                                      double width,
                                                      double height,
                                                      double yaw,
                                                      const std::string& frame,
                                                      const rclcpp::Time& stamp);
    
    rviz_common::properties::FloatProperty* width_prop_;
    rviz_common::properties::FloatProperty* height_prop_;
    rviz_common::properties::FloatProperty* yaw_prop_;
    rviz_common::properties::StringProperty* frame_prop_;
    rviz_common::properties::StringProperty* topic_prop_;

};

}