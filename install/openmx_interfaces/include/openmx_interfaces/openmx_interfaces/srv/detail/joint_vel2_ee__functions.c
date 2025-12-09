// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from openmx_interfaces:srv/JointVel2EE.idl
// generated code does not contain a copyright notice
#include "openmx_interfaces/srv/detail/joint_vel2_ee__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
openmx_interfaces__srv__JointVel2EE_Request__init(openmx_interfaces__srv__JointVel2EE_Request * msg)
{
  if (!msg) {
    return false;
  }
  // q1
  // q2
  // q3
  // q4
  // q1_dot
  // q2_dot
  // q3_dot
  // q4_dot
  return true;
}

void
openmx_interfaces__srv__JointVel2EE_Request__fini(openmx_interfaces__srv__JointVel2EE_Request * msg)
{
  if (!msg) {
    return;
  }
  // q1
  // q2
  // q3
  // q4
  // q1_dot
  // q2_dot
  // q3_dot
  // q4_dot
}

bool
openmx_interfaces__srv__JointVel2EE_Request__are_equal(const openmx_interfaces__srv__JointVel2EE_Request * lhs, const openmx_interfaces__srv__JointVel2EE_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // q1
  if (lhs->q1 != rhs->q1) {
    return false;
  }
  // q2
  if (lhs->q2 != rhs->q2) {
    return false;
  }
  // q3
  if (lhs->q3 != rhs->q3) {
    return false;
  }
  // q4
  if (lhs->q4 != rhs->q4) {
    return false;
  }
  // q1_dot
  if (lhs->q1_dot != rhs->q1_dot) {
    return false;
  }
  // q2_dot
  if (lhs->q2_dot != rhs->q2_dot) {
    return false;
  }
  // q3_dot
  if (lhs->q3_dot != rhs->q3_dot) {
    return false;
  }
  // q4_dot
  if (lhs->q4_dot != rhs->q4_dot) {
    return false;
  }
  return true;
}

bool
openmx_interfaces__srv__JointVel2EE_Request__copy(
  const openmx_interfaces__srv__JointVel2EE_Request * input,
  openmx_interfaces__srv__JointVel2EE_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // q1
  output->q1 = input->q1;
  // q2
  output->q2 = input->q2;
  // q3
  output->q3 = input->q3;
  // q4
  output->q4 = input->q4;
  // q1_dot
  output->q1_dot = input->q1_dot;
  // q2_dot
  output->q2_dot = input->q2_dot;
  // q3_dot
  output->q3_dot = input->q3_dot;
  // q4_dot
  output->q4_dot = input->q4_dot;
  return true;
}

openmx_interfaces__srv__JointVel2EE_Request *
openmx_interfaces__srv__JointVel2EE_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  openmx_interfaces__srv__JointVel2EE_Request * msg = (openmx_interfaces__srv__JointVel2EE_Request *)allocator.allocate(sizeof(openmx_interfaces__srv__JointVel2EE_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(openmx_interfaces__srv__JointVel2EE_Request));
  bool success = openmx_interfaces__srv__JointVel2EE_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
openmx_interfaces__srv__JointVel2EE_Request__destroy(openmx_interfaces__srv__JointVel2EE_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    openmx_interfaces__srv__JointVel2EE_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
openmx_interfaces__srv__JointVel2EE_Request__Sequence__init(openmx_interfaces__srv__JointVel2EE_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  openmx_interfaces__srv__JointVel2EE_Request * data = NULL;

  if (size) {
    data = (openmx_interfaces__srv__JointVel2EE_Request *)allocator.zero_allocate(size, sizeof(openmx_interfaces__srv__JointVel2EE_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = openmx_interfaces__srv__JointVel2EE_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        openmx_interfaces__srv__JointVel2EE_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
openmx_interfaces__srv__JointVel2EE_Request__Sequence__fini(openmx_interfaces__srv__JointVel2EE_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      openmx_interfaces__srv__JointVel2EE_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

openmx_interfaces__srv__JointVel2EE_Request__Sequence *
openmx_interfaces__srv__JointVel2EE_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  openmx_interfaces__srv__JointVel2EE_Request__Sequence * array = (openmx_interfaces__srv__JointVel2EE_Request__Sequence *)allocator.allocate(sizeof(openmx_interfaces__srv__JointVel2EE_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = openmx_interfaces__srv__JointVel2EE_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
openmx_interfaces__srv__JointVel2EE_Request__Sequence__destroy(openmx_interfaces__srv__JointVel2EE_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    openmx_interfaces__srv__JointVel2EE_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
openmx_interfaces__srv__JointVel2EE_Request__Sequence__are_equal(const openmx_interfaces__srv__JointVel2EE_Request__Sequence * lhs, const openmx_interfaces__srv__JointVel2EE_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!openmx_interfaces__srv__JointVel2EE_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
openmx_interfaces__srv__JointVel2EE_Request__Sequence__copy(
  const openmx_interfaces__srv__JointVel2EE_Request__Sequence * input,
  openmx_interfaces__srv__JointVel2EE_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(openmx_interfaces__srv__JointVel2EE_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    openmx_interfaces__srv__JointVel2EE_Request * data =
      (openmx_interfaces__srv__JointVel2EE_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!openmx_interfaces__srv__JointVel2EE_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          openmx_interfaces__srv__JointVel2EE_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!openmx_interfaces__srv__JointVel2EE_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
openmx_interfaces__srv__JointVel2EE_Response__init(openmx_interfaces__srv__JointVel2EE_Response * msg)
{
  if (!msg) {
    return false;
  }
  // vx
  // vy
  // vz
  // wx
  // wy
  // wz
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    openmx_interfaces__srv__JointVel2EE_Response__fini(msg);
    return false;
  }
  return true;
}

void
openmx_interfaces__srv__JointVel2EE_Response__fini(openmx_interfaces__srv__JointVel2EE_Response * msg)
{
  if (!msg) {
    return;
  }
  // vx
  // vy
  // vz
  // wx
  // wy
  // wz
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
openmx_interfaces__srv__JointVel2EE_Response__are_equal(const openmx_interfaces__srv__JointVel2EE_Response * lhs, const openmx_interfaces__srv__JointVel2EE_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // vx
  if (lhs->vx != rhs->vx) {
    return false;
  }
  // vy
  if (lhs->vy != rhs->vy) {
    return false;
  }
  // vz
  if (lhs->vz != rhs->vz) {
    return false;
  }
  // wx
  if (lhs->wx != rhs->wx) {
    return false;
  }
  // wy
  if (lhs->wy != rhs->wy) {
    return false;
  }
  // wz
  if (lhs->wz != rhs->wz) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
openmx_interfaces__srv__JointVel2EE_Response__copy(
  const openmx_interfaces__srv__JointVel2EE_Response * input,
  openmx_interfaces__srv__JointVel2EE_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // vx
  output->vx = input->vx;
  // vy
  output->vy = input->vy;
  // vz
  output->vz = input->vz;
  // wx
  output->wx = input->wx;
  // wy
  output->wy = input->wy;
  // wz
  output->wz = input->wz;
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

openmx_interfaces__srv__JointVel2EE_Response *
openmx_interfaces__srv__JointVel2EE_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  openmx_interfaces__srv__JointVel2EE_Response * msg = (openmx_interfaces__srv__JointVel2EE_Response *)allocator.allocate(sizeof(openmx_interfaces__srv__JointVel2EE_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(openmx_interfaces__srv__JointVel2EE_Response));
  bool success = openmx_interfaces__srv__JointVel2EE_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
openmx_interfaces__srv__JointVel2EE_Response__destroy(openmx_interfaces__srv__JointVel2EE_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    openmx_interfaces__srv__JointVel2EE_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
openmx_interfaces__srv__JointVel2EE_Response__Sequence__init(openmx_interfaces__srv__JointVel2EE_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  openmx_interfaces__srv__JointVel2EE_Response * data = NULL;

  if (size) {
    data = (openmx_interfaces__srv__JointVel2EE_Response *)allocator.zero_allocate(size, sizeof(openmx_interfaces__srv__JointVel2EE_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = openmx_interfaces__srv__JointVel2EE_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        openmx_interfaces__srv__JointVel2EE_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
openmx_interfaces__srv__JointVel2EE_Response__Sequence__fini(openmx_interfaces__srv__JointVel2EE_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      openmx_interfaces__srv__JointVel2EE_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

openmx_interfaces__srv__JointVel2EE_Response__Sequence *
openmx_interfaces__srv__JointVel2EE_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  openmx_interfaces__srv__JointVel2EE_Response__Sequence * array = (openmx_interfaces__srv__JointVel2EE_Response__Sequence *)allocator.allocate(sizeof(openmx_interfaces__srv__JointVel2EE_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = openmx_interfaces__srv__JointVel2EE_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
openmx_interfaces__srv__JointVel2EE_Response__Sequence__destroy(openmx_interfaces__srv__JointVel2EE_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    openmx_interfaces__srv__JointVel2EE_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
openmx_interfaces__srv__JointVel2EE_Response__Sequence__are_equal(const openmx_interfaces__srv__JointVel2EE_Response__Sequence * lhs, const openmx_interfaces__srv__JointVel2EE_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!openmx_interfaces__srv__JointVel2EE_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
openmx_interfaces__srv__JointVel2EE_Response__Sequence__copy(
  const openmx_interfaces__srv__JointVel2EE_Response__Sequence * input,
  openmx_interfaces__srv__JointVel2EE_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(openmx_interfaces__srv__JointVel2EE_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    openmx_interfaces__srv__JointVel2EE_Response * data =
      (openmx_interfaces__srv__JointVel2EE_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!openmx_interfaces__srv__JointVel2EE_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          openmx_interfaces__srv__JointVel2EE_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!openmx_interfaces__srv__JointVel2EE_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
