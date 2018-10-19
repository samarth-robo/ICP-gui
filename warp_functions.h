#ifndef WARP_FUNCTIONS_H
#define WARP_FUNCTIONS_H

#include <pcl/registration/eigen.h>
#include <pcl/registration/warp_point_rigid.h>

namespace pcl
{
  namespace registration
  {
    /** \brief @b Only XY
      */
    template <typename PointSourceT, typename PointTargetT, typename Scalar = float>
    class WarpPointXY : public WarpPointRigid<PointSourceT, PointTargetT, Scalar>
    {
      public:
        typedef typename WarpPointRigid<PointSourceT, PointTargetT, Scalar>::Matrix4 Matrix4;
        typedef typename WarpPointRigid<PointSourceT, PointTargetT, Scalar>::VectorX VectorX;

        typedef boost::shared_ptr<WarpPointXY<PointSourceT, PointTargetT, Scalar> > Ptr;
        typedef boost::shared_ptr<const WarpPointXY<PointSourceT, PointTargetT, Scalar> > ConstPtr;

        /** \brief Constructor. */
        WarpPointXY () : WarpPointRigid<PointSourceT, PointTargetT, Scalar> (2) {}

        /** \brief Empty destructor */
        virtual ~WarpPointXY () {}

        /** \brief Set warp parameters.
          * \param[in] p warp parameters (tx ty)
          */
        virtual void
        setParam (const VectorX & p)
        {
          assert (p.rows () == this->getDimension ());
          Matrix4 &trans = this->transform_matrix_;

          trans = Matrix4::Identity();
          // Copy the rotation and translation components
          trans.block (0, 3, 2, 1) = Eigen::Matrix<Scalar, 2, 1> (p[0], p[1]);
        }
    };

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
      float roll_limit, pitch_limit;

      public:
        typedef typename WarpPointRigid<PointSourceT, PointTargetT, Scalar>::Matrix4 Matrix4;
        typedef typename WarpPointRigid<PointSourceT, PointTargetT, Scalar>::VectorX VectorX;

        typedef boost::shared_ptr<WarpPointNoAzim<PointSourceT, PointTargetT, Scalar> > Ptr;
        typedef boost::shared_ptr<const WarpPointNoAzim<PointSourceT, PointTargetT, Scalar> > ConstPtr;

        /** \brief Constructor. */
        WarpPointNoAzim(float roll_limit=30.f, float pitch_limit=30.f) :
          WarpPointRigid<PointSourceT, PointTargetT, Scalar>(5),
          roll_limit(roll_limit*M_PI/180.f), pitch_limit(pitch_limit*M_PI/180.f) {}

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
          float roll = p[3];
          if (roll < -roll_limit) roll = -roll_limit;
          if (roll > +roll_limit) roll = +roll_limit;
          Eigen::AngleAxis<Scalar> rollAngle (roll, Eigen::Matrix<Scalar, 3, 1>(1, 0, 0));
          float pitch = p[4];
          if (pitch < -pitch_limit) pitch = -pitch_limit;
          if (pitch > +pitch_limit) pitch = +pitch_limit;
          Eigen::AngleAxis<Scalar> pitchAngle(pitch, Eigen::Matrix<Scalar, 3, 1>(0, 1, 0));
          Eigen::Quaternion<Scalar> q = pitchAngle * rollAngle;
          trans.topLeftCorner(3, 3) = q.matrix();
        }
    };

    /** \brief @b Translation + Yaw
     */
    template <typename PointSourceT, typename PointTargetT, typename Scalar = float>
    class WarpPointNoRollPitch : public WarpPointRigid<PointSourceT, PointTargetT, Scalar>
    {
      float yaw_limit;

      public:
        typedef typename WarpPointRigid<PointSourceT, PointTargetT, Scalar>::Matrix4 Matrix4;
        typedef typename WarpPointRigid<PointSourceT, PointTargetT, Scalar>::VectorX VectorX;

        typedef boost::shared_ptr<WarpPointNoRollPitch<PointSourceT, PointTargetT, Scalar> > Ptr;
        typedef boost::shared_ptr<const WarpPointNoRollPitch<PointSourceT, PointTargetT, Scalar> > ConstPtr;

        /** \brief Constructor. */
        WarpPointNoRollPitch(float yaw_limit=30.f) :
          WarpPointRigid<PointSourceT, PointTargetT, Scalar>(4),
          yaw_limit(yaw_limit * M_PI/180.f) {}

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
          float yaw  = p[3];
          if (yaw < -yaw_limit) yaw = -yaw_limit;
          if (yaw > +yaw_limit) yaw = +yaw_limit;
          Eigen::AngleAxis<Scalar> yawAngle (yaw, Eigen::Matrix<Scalar, 3, 1>(0, 0, 1));
          Eigen::Quaternion<Scalar> q(yawAngle);
          trans.topLeftCorner(3, 3) = q.matrix();
        }
    };

    /** \brief @b Full 6D
     */
    template <typename PointSourceT, typename PointTargetT, typename Scalar = float>
    class WarpPoint6D : public WarpPointRigid<PointSourceT, PointTargetT, Scalar>
    {
      float roll_limit, pitch_limit, yaw_limit;

      public:
        typedef typename WarpPointRigid<PointSourceT, PointTargetT, Scalar>::Matrix4 Matrix4;
        typedef typename WarpPointRigid<PointSourceT, PointTargetT, Scalar>::VectorX VectorX;

        typedef boost::shared_ptr<WarpPoint6D<PointSourceT, PointTargetT, Scalar> > Ptr;
        typedef boost::shared_ptr<const WarpPoint6D<PointSourceT, PointTargetT, Scalar> > ConstPtr;

        /** \brief Constructor. */
        WarpPoint6D(float roll_limit=30.f, float pitch_limit=30.f, float yaw_limit=30.f) :
          WarpPointRigid<PointSourceT, PointTargetT, Scalar>(6),
          roll_limit(roll_limit * M_PI/180.f),
          pitch_limit(pitch_limit * M_PI/180.f),
          yaw_limit(yaw_limit * M_PI/180.f) {}

        /** \brief Empty destructor */
        virtual ~WarpPoint6D () {}

        /** \brief Set warp parameters.
          * \param[in] p warp parameters (tx ty tz rx ry rz)
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
          float roll  = p[3];
          if (roll < -roll_limit) roll = -roll_limit;
          if (roll > +roll_limit) roll = +roll_limit;
          float pitch  = p[4];
          if (pitch < -pitch_limit) pitch = -pitch_limit;
          if (pitch > +pitch_limit) pitch = +pitch_limit;
          float yaw  = p[5];
          if (yaw < -yaw_limit) yaw = -yaw_limit;
          if (yaw > +yaw_limit) yaw = +yaw_limit;
          Eigen::AngleAxis<Scalar> rollAngle (roll, Eigen::Matrix<Scalar, 3, 1>(1, 0, 0));
          Eigen::AngleAxis<Scalar> pitchAngle (pitch, Eigen::Matrix<Scalar, 3, 1>(0, 1, 0));
          Eigen::AngleAxis<Scalar> yawAngle (yaw, Eigen::Matrix<Scalar, 3, 1>(0, 0, 1));
          Eigen::Quaternion<Scalar> q = pitchAngle * rollAngle * yawAngle;
          trans.topLeftCorner(3, 3) = q.matrix();
        }
    };
  }
}

#endif // WARP_FUNCTIONS_H
