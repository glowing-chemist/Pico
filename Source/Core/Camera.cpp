#include "Core/Camera.hpp"

#include <glm/gtx/transform.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


namespace Scene
{


    void Camera::moveForward(const float distance)
    {
        mPosition += distance * mDirection;
    }


    void Camera::moveBackward(const float distance)
    {
        mPosition -= distance * mDirection;
    }


    void Camera::moveLeft(const float distance)
    {
        mPosition -= distance * getRight();
    }


    void Camera::moveRight(const float distance)
    {
        mPosition += distance * getRight();
    }


    void Camera::moveUp(const float distance)
    {
        mPosition += mUp * distance;
    }


    void Camera::moveDown(const float distance)
    {
        mPosition -= mUp * distance;
    }


    void Camera::rotatePitch(const float angle)
    {
        const glm::vec3 rotationAxis = getRight();
        const glm::mat3 rotation = glm::rotate(glm::radians(angle), rotationAxis);
        mDirection = glm::normalize(rotation * mDirection);
        mUp = glm::normalize(rotation * mUp);
    }


    void Camera::rotateYaw(const float angle)
    {
        const glm::mat3 rotation = glm::rotate(glm::radians(angle), mUp);
        mDirection = glm::normalize(rotation * mDirection);
    }


    void Camera::rotateWorldUp(const float angle)
    {
        const glm::mat3 rotation = glm::rotate(glm::radians(angle), glm::vec3(0.0f, -1.0f, 0.0f));
        mDirection = glm::normalize(rotation * mDirection);
        mUp = glm::normalize(rotation * mUp);
    }


    glm::mat4x4 Camera::getViewMatrix() const
    {
        return glm::lookAt(mPosition, mPosition + mDirection, mUp);
    }


    glm::mat4x4 Camera::getProjectionMatrix() const
    {
        return glm::perspective(glm::radians(mFieldOfView), mAspect, mFarPlaneDistance, mNearPlaneDistance);
    }

}
