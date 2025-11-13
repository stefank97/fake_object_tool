#include "fake_object_tool/rectangle_tool.hpp"
#include <rviz_common/tool.hpp>                   
#include <rviz_common/display_context.hpp>          
#include <rviz_common/viewport_mouse_event.hpp>    
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rclcpp/rclcpp.hpp>                        
#include <geometry_msgs/msg/point_stamped.hpp>      
#include <geometry_msgs/msg/polygon_stamped.hpp>    
#include <cmath>                                   
#include <rviz_common/view_manager.hpp>
#include <rviz_common/render_panel.hpp>
#include <OgreRay.h>
#include <OgreCamera.h>
#include <pluginlib/class_list_macros.hpp>



namespace fake_object_tool {

    RectangleTool::RectangleTool()
    {
        shortcut_key_ = 'o';
        auto parent = getPropertyContainer();
        //Tool-Properties
        width_prop_ = new rviz_common::properties::FloatProperty("Width (m)", 1.5, "Set the width of the rectangle", parent);
        height_prop_ = new rviz_common::properties::FloatProperty("Height (m)", 1.0, "Set the height of the rectangle", parent);
        yaw_prop_ = new rviz_common::properties::FloatProperty("Yaw (deg)", 0.0, "Set the rotation of the rectangle in degree", parent);
        frame_prop_ = new rviz_common::properties::StringProperty("Frame", "map", "Coordinate-frame for the rectangle", parent);
        topic_prop_ = new rviz_common::properties::StringProperty("Topic", "/fake_object/rectangle", "The topic to be published", parent);

    };

    void RectangleTool::onInitialize(){
        topic_prop_->setReadOnly(true);
        auto node_ptr_ = context_->getRosNodeAbstraction().lock();
        rclcpp::Node::SharedPtr node_ = node_ptr_->get_raw_node();

        std::string topic = topic_prop_->getStdString();
        pub_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>(topic, rclcpp::QoS(10));
        RCLCPP_DEBUG(
            rclcpp::get_logger("rviz"),
            "Topic for rectangle publication is: %s",
            topic.c_str()
        );


    }

    void RectangleTool::activate() {}
    void RectangleTool::deactivate() {}


    geometry_msgs::msg::PolygonStamped RectangleTool::buildRectangle(const geometry_msgs::msg::Point& center,
                        double width,
                        double height,
                        double yaw,
                        const std::string& frame,
                        const rclcpp::Time& stamp){

        const double central_x = center.x;
        const double central_y = center.y;
        const double half_width = width / 2;
        const double half_height = height / 2;

        RCLCPP_DEBUG(
            rclcpp::get_logger("rviz"),
            "Center point for rectangle is (%f, %f)", 
            central_x, central_y
        );

        const double cos = std::cos(yaw);
        const double sin = std::sin(yaw);

        geometry_msgs::msg::PolygonStamped polygon;
        geometry_msgs::msg::Point32 p;

        polygon.header.frame_id = frame;
        polygon.header.stamp = stamp;

        RCLCPP_DEBUG(
            rclcpp::get_logger("rviz"),
            "Frame for rectangle publication is: %s; at stamp %u.%u",
            frame.c_str(), stamp.sec, stamp.nanosec
        );
        
        //Delete old points (Just to be sure)
        polygon.polygon.points.clear();

        //Rotation-matrix + central point bc its not in the origin
        //Point A (+half_width,+half_height)
        p.x = central_x + cos * half_width - sin * half_height;
        p.y = central_y + sin * half_width + cos * half_height;
        p.z = 1.0;
        polygon.polygon.points.push_back(p);


        //point B (-half_width,+half_height)
        p.x = central_x + cos * -half_width - sin * half_height;
        p.y = central_y + sin * -half_width + cos * half_height;
        p.z = 1.0;
        polygon.polygon.points.push_back(p);


        //Point C (-half_width,-half_height)
        p.x = central_x + cos * -half_width - sin * -half_height;
        p.y = central_y + sin * -half_width + cos * -half_height;
        p.z = 1.0;
        polygon.polygon.points.push_back(p);


        //Point D (+half_width,-half_height)
        p.x = central_x + cos * half_width - sin * -half_height;
        p.y = central_y + sin * half_width + cos * -half_height;
        p.z = 1.0;
        polygon.polygon.points.push_back(p);

        RCLCPP_DEBUG(
            rclcpp::get_logger("rviz"),
            "Built Rectangle for points: A=(%f,%f), B=(%f,%f), C=(%f,%f), D=(%f,%f)", 
            polygon.polygon.points[0].x, polygon.polygon.points[0].y,
            polygon.polygon.points[1].x, polygon.polygon.points[1].y,
            polygon.polygon.points[2].x, polygon.polygon.points[2].y,
            polygon.polygon.points[3].x, polygon.polygon.points[3].y
        );

        return polygon;
    }

    int RectangleTool::processMouseEvent(rviz_common::ViewportMouseEvent& event){
        //++++AI Code++++ https://claude.ai/share/f9c58384-b464-4604-a71d-3cd694431aba
        // Only react to left mouse button down events
        if (!event.leftDown()) {
            return rviz_common::Tool::Render;
        }

        //Get Viewport-Controller and Camera
        auto view_controller = context_->getViewManager()->getCurrent();
        if (!view_controller) {
            return rviz_common::Tool::Render;
        }

        Ogre::Camera* camera = view_controller->getCamera();
        if (!camera) {
            return rviz_common::Tool::Render;
        }

        //Get Viewport-Dimensions from RenderPanel
        int render_width = event.panel->width();
        int render_height = event.panel->height();

        // Normalise Viewport-Coordinates (0.0 - 1.0)
        float norm_x = static_cast<float>(event.x) / static_cast<float>(render_width);
        float norm_y = static_cast<float>(event.y) / static_cast<float>(render_height);

        // Create Ray from Camera through Mouse-Position
        Ogre::Ray mouse_ray = camera->getCameraToViewportRay(norm_x, norm_y);

        //  Calculate cross product with level z=0 (Ground Plane)
        Ogre::Vector3 origin = mouse_ray.getOrigin();
        Ogre::Vector3 direction = mouse_ray.getDirection();
        
        // Calculate t for z = 0: origin.z + t * direction.z = 0
        if (std::abs(direction.z) < 0.0001) {
            // Ray is parallel to ground plane
            return rviz_common::Tool::Render;
        }
        //Calculate parameter t (Distance on the beam) and intersection point
        float t = -origin.z / direction.z;
        Ogre::Vector3 intersection = origin + t * direction;

        // Set the point for rectangle center with the cross product result
        geometry_msgs::msg::Point center;
        center.x = intersection.x;  
        center.y = intersection.y;
        center.z = 0.0;
        RCLCPP_DEBUG(
            rclcpp::get_logger("rviz"),
            "Cross product at (%f, %f, %f)",
            center.x, center.y, center.z
        )
        // Get current time for the message stamp
        auto raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
        const rclcpp::Time stamp = raw_node->now();
                            //++++AI Code++++ 

        const double width = static_cast<double>(width_prop_->getFloat());
        const double height = static_cast<double>(height_prop_->getFloat());
        const double yaw_deg = static_cast<double>(yaw_prop_->getFloat());
        const double yaw = yaw_deg * M_PI / 180.0; // Convert to radians
        const std::string frame = frame_prop_->getStdString();


        RCLCPP_DEBUG(
            rclcpp::get_logger("rviz"),
            "Rectangle properties - Width: %f, Height: %f, Yaw: %f degrees, Frame: %s",
            width, height, yaw_deg, frame.c_str()
        );


        auto polygon = buildRectangle(center,width, height, yaw, frame, stamp);
        RCLCPP_INFO(
            rclcpp::get_logger("rviz"),
            "Building rectangle at (%2.f, %2.f) with width %2.f, height %2.f, yaw %2.f degrees in frame %s",
            center.x, center.y, width, height, yaw_deg, frame.c_str()
        );
        pub_->publish(polygon);

        return rviz_common::Tool::Render | rviz_common::Tool::Finished; //Update the rviz Render and finish the tool
    }

    
}
PLUGINLIB_EXPORT_CLASS(fake_object_tool::RectangleTool, rviz_common::Tool)