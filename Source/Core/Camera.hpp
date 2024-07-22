#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "Core/AABB.hpp"

#include "glm/common.hpp"
#include "glm/mat4x4.hpp"

namespace Scene
{

    class Camera
    {
    public:

        Camera(const glm::vec3& position,
            const glm::vec3& direction,
            const float aspect,
            float nearPlaneDistance = 0.1f,
            float farPlaneDistance = 10.0f,
            float fieldOfView = 90.0f)
            :   mPosition{ position },
                mDirection{ direction },
                mUp{ 0.0f, 1.0f, 0.0f },
                mResolution{1902, 1080},
                mAspect{ aspect },
                mNearPlaneDistance{nearPlaneDistance},
                mFarPlaneDistance{farPlaneDistance},
                mFieldOfView{fieldOfView} {}


        void moveForward(const float distance);
        void moveBackward(const float distance);
        void moveLeft(const float distance);
        void moveRight(const float distance);
        void moveUp(const float distance);
        void moveDown(const float distance);

        void rotatePitch(const float);
        void rotateYaw(const float);
        void rotateWorldUp(const float);

        Core::Ray generate_ray(const glm::vec2& Xi, const glm::uvec2& pix) const;

        const glm::vec3& getPosition() const
            { return mPosition; }

        void setPosition(const glm::vec3& pos)
            { mPosition = pos; }

        const glm::vec3& getDirection() const
            { return mDirection; }

        void setDirection(const glm::vec3& dir)
            { mDirection = dir; }

        void setUp(const glm::vec3& up)
            { mUp = up; }

        const glm::vec3 getUp() const
            { return mUp; }

        // Get the vector perpendicular to the direction vector (rotated 90 degrees clockwise)
        glm::vec3 getRight() const
        { return glm::cross(glm::normalize(mDirection), mUp); }

        void setNearPlane(const float nearDistance)
            { mNearPlaneDistance = nearDistance; }

        void setFarPlane(const float farDistance)
            { mFarPlaneDistance = farDistance; }

        void setFOVDegrees(const float fov)
            { mFieldOfView = fov; }

        void setAspect(const float aspect)
            { mAspect = aspect; }

        float getAspect() const
        { return mAspect; }

        void set_resolution(const glm::uvec2& res)
        { mResolution = res; }

        glm::mat4x4 getViewMatrix() const;
        glm::mat4x4 getProjectionMatrix() const;

        float getNearPlane() const
        { return mNearPlaneDistance; }

        float getFarPlane() const
        { return mFarPlaneDistance; }

        float getFOV() const
        { return mFieldOfView; }


    private:

        glm::vec3 mPosition;
        glm::vec3 mDirection;
        glm::vec3 mUp;
        glm::uvec2 mResolution;
        float mAspect;
        float mNearPlaneDistance;
        float mFarPlaneDistance;
        float mFieldOfView;
    };

}

#endif
