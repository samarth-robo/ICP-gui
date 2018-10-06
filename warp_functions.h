#ifndef WARP_FUNCTIONS_H
#define WARP_FUNCTIONS_H

#include <pcl/registration/eigen.h>
#include <pcl/registration/warp_point_rigid.h>

namespace pcl
{
  namespace registration
  {
    /** \brief @b No rotation
      */
    template <typename PointSourceT, typename PointTargetT, typename Scalar = float>
    class WarpPointNoRotation : public WarpPointRigid<PointSourceT, PointTargetT, Scalar>
    {
      public:
        typedef typename WarpPointRigid<PointSourceT, PointTargetT, Scalar>::Matrix4 Matrix4;
        typedef typename WarpPointRigid<PointSourceT, PointTargetT, Scalar>::VectorX VectorX;

        typedef boost::shared_ptr<WarpPointNoRotation<PointSourceT, PointTargetT, Scalar> > Ptr;
        typedef boost::shared_ptr<const WarpPointNoRotation<PointSourceT, PointTargetT, Scalar> > ConstPtr;

        /** \brief Constructor. */
        WarpPointNoRotation () : WarpPointRigid<PointSourceT, PointTargetT, Scalar> (3) {}

        /** \brief Empty destructor */
        virtual ~WarpPointNoRotation () {}

        /** \brief Set warp parameters.
          * \param[in] p warp parameters (tx ty tz)
          */
        virtual void
        setParam (const VectorX & p)
        {
          assert (p.rows () == this->getDimension ());
          Matrix4 &trans = this->transform_matrix_;

          trans = Matrix4::Identity();
          // Copy the rotation and translation components
          trans.block (0, 3, 3, 1) = Eigen::Matrix<Scalar, 3, 1> (p[0], p[1], p[2]);
        }
    };

    /** \brief @b Translation + Roll Pitch
     */
    template <typename PointSourceT, typename PointTargetT, typename Scalar = float>
    class WarpPointNoAzim : public WarpPointRigid<PointSourceT, PointTargetT, Scalar>
    {
      public:
        typedef typename WarpPointRigid<PointSourceT, PointTargetT, Scalar>::Matrix4 Matrix4;
        typedef typename WarpPointRigid<PointSourceT, PointTargetT, Scalar>::VectorX VectorX;

        typedef boost::shared_ptr<WarpPointNoAzim<PointSourceT, PointTargetT, Scalar> > Ptr;
        typedef boost::shared_ptr<const WarpPointNoAzim<PointSourceT, PointTargetT, Scalar> > ConstPtr;

        /** \brief Constructor. */
        WarpPointNoAzim () : WarpPointRigid<PointSourceT, PointTargetT, Scalar> (5) {}

        /** \brief Empty destructor */
        virtual ~WarpPointNoAzim () {}

        /** \brief Set warp parameters.
          * \param[in] p warp parameters (tx ty tz rx ry)
          */
        virtual void
        setParam (const VectorX & p)
        {
          assert (p.rows () == this->getDimension ());
          Matrix4 &trans = this->transform_matrix_;

          trans = Matrix4::Identity();
          // Copy the rotation and translation components
          trans.block (0, 3, 3, 1) = Eigen::Matrix<Scalar, 3, 1> (p[0], p[1], p[2]);
          // rotation
          Eigen::AngleAxis<Scalar> rollAngle (p[3], Eigen::Matrix<Scalar, 3, 1>(1, 0, 0));
          Eigen::AngleAxis<Scalar> pitchAngle(p[4], Eigen::Matrix<Scalar, 3, 1>(0, 1, 0));
          Eigen::Quaternion<Scalar> q = pitchAngle * rollAngle;
          trans.topLeftCorner(3, 3) = q.matrix();
        }
    };

    /** \brief @b Translation + Yaw
     */
    template <typename PointSourceT, typename PointTargetT, typename Scalar = float>
    class WarpPointNoRollPitch : public WarpPointRigid<PointSourceT, PointTargetT, Scalar>
    {
      public:
        typedef typename WarpPointRigid<PointSourceT, PointTargetT, Scalar>::Matrix4 Matrix4;
        typedef typename WarpPointRigid<PointSourceT, PointTargetT, Scalar>::VectorX VectorX;

        typedef boost::shared_ptr<WarpPointNoRollPitch<PointSourceT, PointTargetT, Scalar> > Ptr;
        typedef boost::shared_ptr<const WarpPointNoRollPitch<PointSourceT, PointTargetT, Scalar> > ConstPtr;

        /** \brief Constructor. */
        WarpPointNoRollPitch () : WarpPointRigid<PointSourceT, PointTargetT, Scalar> (4) {}

        /** \brief Empty destructor */
        virtual ~WarpPointNoRollPitch () {}

        /** \brief Set warp parameters.
          * \param[in] p warp parameters (tx ty tz rz)
          */
        virtual void
        setParam (const VectorX & p)
        {
          assert (p.rows () == this->getDimension ());
          Matrix4 &trans = this->transform_matrix_;

          trans = Matrix4::Identity();
          // Copy the rotation and translation components
          trans.block (0, 3, 3, 1) = Eigen::Matrix<Scalar, 3, 1> (p[0], p[1], p[2]);
          // rotation
          Eigen::AngleAxis<Scalar> yawAngle (p[3], Eigen::Matrix<Scalar, 3, 1>(0, 0, 1));
          Eigen::Quaternion<Scalar> q(yawAngle);
          trans.topLeftCorner(3, 3) = q.matrix();
        }
    };
  }
}

#endif // WARP_FUNCTIONS_H
