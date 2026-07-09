#ifndef ROSARIA2_INCLUDE_ROSARIA2_DYNAMICPARAMETER_HPP_
#define ROSARIA2_INCLUDE_ROSARIA2_DYNAMICPARAMETER_HPP_

#include <memory>
#include <utility>
#include <string>
#include <type_traits>

#ifdef DYNAMICPARAMETER_LOG_PARAMETER_UPDATE
#include <sstream>
#endif  // DYNAMICPARAMETER_LOG_PARAMETER_UPDATE

#include <rclcpp/rclcpp.hpp>

namespace rclcpp {

//------------------------------------------------------------------------------
/// @brief      Class template that wraps around ROS2's parameter event/callback routines.
///             Provides a single object whose value is self-updated and is implicitly convertible (can replace) to the parameter type T.
///
/// @tparam     T       Parameter type.
///
/// @note       Restricts parameter event handling to class definition, leading to more clean/verbose code when employed.
///
/// @note       Declares parameter on ROS (under parent node), no need to initialize parameter before instantiation.
///
/// @note       Extensible type.
///
/// @todo       Add support for parameter descriptor argument when declaring parameter.
///
template < typename T >
class DynamicParameterView {
 public:
    //--------------------------------------------------------------------------
    /// @brief      Pointer type alias.
    ///
    using SharedPtr = std::shared_ptr< DynamicParameterView< T > >;

    //--------------------------------------------------------------------------
    /// @brief      Constructs a new instance.
    ///
    /// @param[in]  node       Node on which to declare parameter.
    /// @param[in]  name       Parameter name.
    /// @param[in]  args       Parameter initial value or constructor arguments forward to T(Args...)
    ///
    /// @tparam     Args       Variadic parameter pack encoding argument types
    ///
    /// @note       Arguments define the *default* parameter value; if ROS parameter is already set, value is kept.
    ///
    template < typename... Args, typename = typename std::enable_if_t< std::is_constructible_v< T, Args... > > >
    DynamicParameterView(rclcpp::Node* node, const std::string& name, Args&&... args);

    //--------------------------------------------------------------------------
    /// @brief      Constructs a new instance.
    ///
    /// @param[in]  node                Node on which to declare parameter.
    /// @param[in]  event_handler       Parameter event handler to add parameter subscription to.
    /// @param[in]  name                Parameter name.
    /// @param[in]  args                Parameter initial value or constructor arguments forward to T(Args...)
    ///
    /// @tparam     Args                Variadic parameter pack encoding argument types
    ///
    /// @note       Constructor overload provided to bypass unecessary instantiation of rclcpp::ParameterEventHandler per DynamicParameterView instance.
    ///             Allows the use of an existing event handler which can be shared among different DynamicParameterView(s).
    ///
    template < typename... Args, typename = typename std::enable_if_t< std::is_constructible_v< T, Args... > > >
    DynamicParameterView(rclcpp::Node* node, std::shared_ptr< rclcpp::ParameterEventHandler > event_handler, const std::string& name, Args&&... args);

    //--------------------------------------------------------------------------
    /// @brief      Destroys the object.
    ///
    virtual ~DynamicParameterView() = default;

    //--------------------------------------------------------------------------
    /// @brief      Parameter value acessor.
    ///
    /// @return     Const reference to (current) parameter value.
    ///
    virtual const T& get() const;

    //--------------------------------------------------------------------------
    /// @brief      Conversion operator to T (as reference).
    ///
    /// @return     Const reference to (current) parameter value.
    ///
    operator const T&() const;

 protected:
    //--------------------------------------------------------------------------
    /// @brief      Updates the parameter value.
    ///
    /// @param[in]  param  Parameter instance with new value.
    ///
    virtual void update(const rclcpp::Parameter& param);

    //--------------------------------------------------------------------------
    /// @brief      Parameter value buffer.
    ///
    T _value;

    //--------------------------------------------------------------------------
    /// @brief      Event handler.
    ///
    /// @note       Parameter updates are only detected during its lifetime.
    ///
    /// @todo       Profile overhead caused by initializing a new event handler on each instance.
    ///
    std::shared_ptr< rclcpp::ParameterEventHandler > _event_handler;

    //--------------------------------------------------------------------------
    /// @brief      Callback handler.
    ///
    /// @note       update() is only called on each parameter change during its lifetime.
    ///
    std::shared_ptr< rclcpp::ParameterCallbackHandle > _handle;
};



//------------------------------------------------------------------------------
/// @cond

template < typename T >
template < typename... Args, typename >
DynamicParameterView< T >::DynamicParameterView(rclcpp::Node* node, std::shared_ptr< rclcpp::ParameterEventHandler > event_handler, const std::string& name, Args&&... args) :
    // _value(std::forward< Args >(args)...),
    _event_handler(event_handler) {
        // declare parameter
        if (node) {
            _value = node->declare_parameter(name, T(std::forward< Args >(args)...));
        } else {
            RCLCPP_WARN(rclcpp::get_logger(name), "Can't initialize parameter view: Invalid node");
        }
        // register callback
        if (_event_handler) {
            _handle = _event_handler->add_parameter_callback(name, std::bind(&DynamicParameterView::update, this, std::placeholders::_1));
        } else {
            RCLCPP_WARN(rclcpp::get_logger(name), "Can't initialize parameter view: Invalid parameter even handler");
        }
}


template < typename T >
template < typename... Args, typename >
DynamicParameterView< T >::DynamicParameterView(rclcpp::Node* node, const std::string& name, Args&&... args) :
    DynamicParameterView(node, std::make_shared< rclcpp::ParameterEventHandler >(node), name, std::forward< Args >(args)...) {
        /* ... */
}


template < typename T >
void DynamicParameterView< T >::update(const rclcpp::Parameter& param) {
    _value = param.get_value< T >();

    #ifdef DYNAMICPARAMETER_LOG_PARAMETER_UPDATE
    std::stringstream ss;
    ss << _value;
    // RCLCPP_INFO(rclcpp::get_logger(param.get_name()), "new value: %s", ss.str().data());                                // param-specific logger
    RCLCPP_INFO(rclcpp::get_logger(_handle->node_name), "%s new value: %s", param.get_name().data(), ss.str().data());  // node-specific
    #endif  // DYNAMICPARAMETER_LOG_PARAMETER_UPDATE
}


template < typename T >
const T& DynamicParameterView< T >::get() const {
    return _value;
}


template < typename T >
DynamicParameterView< T >::operator const T& () const {
    return this->get();
}

/// @endcond
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
/// @brief      Class template that wraps around ROS2's parameter event/callback routines *and* parameter setting.
///             Provides a single object whose value is:
///                 1) self-updated (reflecting ROS parameter updates);
///                 1) implicitly convertible (can replace) to the parameter type T;
///                 2) assignable from instances of T (ROS parameter is updated on the parameter server).
///             Extends DynamicParameterView< > with implementation of parameter setting.
///
/// @tparam     T     Parameter type
///
template < typename T >
class DynamicParameter : public DynamicParameterView< T > {
 public:
    //--------------------------------------------------------------------------
    /// @brief      Pointer type alias.
    ///
    using SharedPtr = std::shared_ptr< DynamicParameter< T > >;

    //--------------------------------------------------------------------------
    /// @brief      Constructs a new instance.
    ///
    /// @param[in]  node       Node on which to declare parameter.
    /// @param[in]  name       Parameter name.
    /// @param[in]  args       Parameter initialvalue or constructor arguments forward to T(Args...)
    ///
    /// @tparam     Args       Variadic parameter pack encoding argument types
    ///
    template < typename... Args, typename = typename std::enable_if_t< std::is_constructible_v< T, Args... > > >
    DynamicParameter(rclcpp::Node* node, const std::string& name, Args&&... args);

    //--------------------------------------------------------------------------
    /// @brief      Constructs a new instance.
    ///
    /// @param[in]  node                Node on which to declare parameter.
    /// @param[in]  event_handler       Parameter event handler to add parameter subscription to.
    /// @param[in]  name                Parameter name.
    /// @param[in]  args                Parameter initialvalue or constructor arguments forward to T(Args...)
    ///
    /// @tparam     Args                Variadic parameter pack encoding argument types
    ///
    /// @note       Constructor overload provided to bypass unecessary instantiation of rclcpp::ParameterEventHandler per DynamicParameter instance.
    ///             Allows the use of an existing event handler which can be shared among different DynamicParameter(s).
    ///
    template < typename... Args, typename = typename std::enable_if_t< std::is_constructible_v< T, Args... > > >
    DynamicParameter(rclcpp::Node* node, std::shared_ptr< rclcpp::ParameterEventHandler > event_handler, const std::string& name, Args&&... args);

    //--------------------------------------------------------------------------
    /// @brief      Assignment operator.
    ///
    /// @param[in]  value  Value to assign.
    ///
    /// @return     *const* reference to value buffer.
    ///
    /// @note       Provides bidirectional operation; parameter value is updated on ROS parameter server.
    ///
    const T& operator=(const T& value);

 protected:
    //--------------------------------------------------------------------------
    /// @brief      (Parent) node.
    ///
    /// @note       Required to implicitely set parameter values.
    ///             Raw pointer is used, no need to share ownership of node.
    ///
    rclcpp::Node* _node;
};


//------------------------------------------------------------------------------
/// @cond

template < typename T >
template < typename... Args, typename >
DynamicParameter< T >::DynamicParameter(rclcpp::Node* node, std::shared_ptr< rclcpp::ParameterEventHandler > event_handler, const std::string& name, Args&&... args) :
    DynamicParameterView< T >::DynamicParameterView(node, event_handler, name, std::forward< Args >(args)...),
    _node(node) {
        /* ... */
}


template < typename T >
template < typename... Args, typename >
DynamicParameter< T >::DynamicParameter(rclcpp::Node* node, const std::string& name, Args&&... args) :
    DynamicParameter(node, std::make_shared< rclcpp::ParameterEventHandler >(node), name, std::forward< Args >(args)...) {
        /* ... */
}


template < typename T >
const T& DynamicParameter< T >::operator=(const T& value) {
    // set parameter on ROS parameter server
    _node->set_parameter(rclcpp::Parameter(DynamicParameter< T >::_handle->parameter_name, value));

    #ifdef DYNAMICPARAMETER_LOG_PARAMETER_UPDATE
    std::stringstream ss;
    ss << _value;
    // RCLCPP_INFO(rclcpp::get_logger(param.get_name()), "new value: %s", ss.str().data());                // param-specific logger
    RCLCPP_INFO(_node->get_logger(), "'%s' new value: %s", param.get_name().data(), ss.str().data());   // node-specific
    #endif  // DYNAMICPARAMETER_LOG_PARAMETER_UPDATE

    return DynamicParameter< T >::_value;
}

/// @endcond
//------------------------------------------------------------------------------

}  // namespace rclcpp

#endif  // ROSARIA2_INCLUDE_ROSARIA2_DYNAMICPARAMETER_HPP_
