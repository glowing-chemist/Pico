#include "Core/Camera.hpp"

#include <glm/gtx/transform.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


namespace Scene
{

    Core::Ray Camera::generate_ray(const glm::vec2& rand, const glm::uvec2& pix) const
    {
        const glm::vec2 offset = glm::vec2(0.5f, 0.5f);
        const glm::vec2 remapped_rand = rand / 2.0f;

        glm::vec3 dir = {((float(pix.x + remapped_rand.x) / float(mResolution.x)) - offset.x) * mAspect, (float(pix.y + remapped_rand.y) / float(mResolution.y)) - offset.y, 1.0f};
        dir = glm::normalize((dir.z * getDirection()) + (dir.y * getUp()) + (dir.x * getRight()));

        Core::Ray ray;
        ray.mDirection = dir;
        ray.mOrigin = glm::vec4(getPosition(), 1.0f);
        ray.mLenght = getFarPlane();
        ray.push_index_of_refraction(1.0f);
        ray.m_payload = glm::vec3(0.0f, 0.0f, 0.0f);
        ray.m_throughput = glm::vec3(1.0f, 1.0f, 1.0f);

        return ray;
    }

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
