#![forbid(unsafe_code)]

use std::fmt::{Display, Debug};
use std::ops::{Add, Mul, Sub, Div, Neg};
use num_traits::{Zero, One, Float};


#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion<T> {
    w: T,
    x: T,
    y: T,
    z: T,
}

impl<T> Quaternion<T>
where
    T: Copy
{
    #[inline]
    pub fn new(w: T, x: T, y: T, z: T) -> Self {
        Quaternion { w, x, y, z }
    }

    #[inline]
    pub fn w(&self) -> T { self.w }

    #[inline]
    pub fn x(&self) -> T { self.x }

    #[inline]
    pub fn y(&self) -> T { self.y }

    #[inline]
    pub fn z(&self) -> T { self.z }
}

impl<T> Add for Quaternion<T>
where
    T: Add<Output = T> + Copy
{ 
    type Output = Quaternion<T>;
    #[inline]
    fn add(self, rhs: Quaternion<T>) -> Quaternion<T> {
        Quaternion {
            w: self.w + rhs.w,
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl<T> Sub for Quaternion<T>
where
    T: Sub<Output = T> + Copy
{
    type Output = Quaternion<T>;
    #[inline]
    fn sub(self, rhs: Quaternion<T>) -> Quaternion<T> {
        Quaternion {
            w: self.w - rhs.w,
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl<T> Mul for Quaternion<T>
where
    T: Add<Output = T> + Sub<Output = T> + Mul<Output = T> + Copy
{
    type Output = Quaternion<T>;
    #[inline]
    fn mul(self, rhs: Quaternion<T>) -> Quaternion<T> {
        Quaternion {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        }
    }
}

impl<T> Mul<T> for Quaternion<T>
where
    T: Mul<Output = T> + Copy
{
    type Output = Quaternion<T>;
    #[inline]
    fn mul(self, scalar: T) -> Quaternion<T> {
        Quaternion {
            w: self.w * scalar,
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

impl<T> Div<T> for Quaternion<T>
where
    T: Div<Output = T> + Copy
{
    type Output = Quaternion<T>;
    #[inline]
    fn div(self, scalar: T) -> Quaternion<T> {
        Quaternion {
            w: self.w / scalar,
            x: self.x / scalar,
            y: self.y / scalar,
            z: self.z / scalar,
        }
    }
}

impl<T> Quaternion<T>
where
    T: Add<Output = T> + Sub<Output = T> + Mul<Output = T> + Neg<Output = T> + Zero + One + Copy + Display
{
    #[inline]
    pub fn dot(self, other: Quaternion<T>) -> T {
        self.w * other.w + self.x * other.x + self.y * other.y + self.z * other.z
    }
    pub fn dot_v(self, other: Quaternion<T>) -> Result<T, String> {
        if !self.w.is_zero() || !other.w.is_zero() {
            Err("Dot for Vector: Non-vector quaternion provided.".to_string())
        } else {
            Ok(self.dot(other))
        }
    }
    #[inline]
    pub fn conjugate(self) -> Quaternion<T> {
        Quaternion {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
    pub fn mat(self) -> [[T; 3]; 3] {
        let two = T::one() + T::one();
        let ww = self.w * self.w;
        let xx = self.x * self.x;
        let yy = self.y * self.y;
        let zz = self.z * self.z;
        let xy = self.x * self.y * two;
        let xz = self.x * self.z * two;
        let yz = self.y * self.z * two;
        let wx = self.w * self.x * two;
        let wy = self.w * self.y * two;
        let wz = self.w * self.z * two;

        [
            [ww + xx - yy - zz, xy - wz, xz + wy],
            [xy + wz, ww - xx + yy - zz, yz - wx],
            [xz - wy, yz + wx, ww - xx - yy + zz],
        ]
    }
    pub fn mat_homogeneous(self) -> [[T; 4]; 4] {
        let two = T::one() + T::one();
        let ww = self.w * self.w;
        let xx = self.x * self.x;
        let yy = self.y * self.y;
        let zz = self.z * self.z;
        let xy = self.x * self.y * two;
        let xz = self.x * self.z * two;
        let yz = self.y * self.z * two;
        let wx = self.w * self.x * two;
        let wy = self.w * self.y * two;
        let wz = self.w * self.z * two;

        [
            [ww + xx - yy - zz, xy - wz, xz + wy, T::zero()],
            [xy + wz, ww - xx + yy - zz, yz - wx, T::zero()],
            [xz - wy, yz + wx, ww - xx - yy + zz, T::zero()],
            [T::zero(), T::zero(), T::zero(), T::one()],
        ]
    }
    pub fn display(self) -> String {
        format!("{} + {}i + {}j + {}k", self.w, self.x, self.y, self.z)
    }
}

impl<T> Quaternion<T>
where
    T: Float + Display + Debug
{
    #[inline]
    pub fn norm(self) -> T {
        (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }
    pub fn is_unit(self, epsilon: T) -> bool {
        (self.norm() - T::one()).abs() < epsilon
    }
    pub fn normalize(self) -> Result<Quaternion<T>, String> {
        let norm = self.norm();
        if norm.is_zero() {
            Err("Normalize: Cannot divide by zero".to_string())
        } else {
            Ok(self / norm)
        }
    }
    pub fn from_euler(roll: T, pitch: T, yaw: T) -> Quaternion<T> {
        let half = T::from(0.5).unwrap();
        let cr = (roll * half).cos();
        let sr = (roll * half).sin();
        let cp = (pitch * half).cos();
        let sp = (pitch * half).sin();
        let cy = (yaw * half).cos();
        let sy = (yaw * half).sin();

        Quaternion {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
        }
    }
    pub fn inverse(self) -> Result<Quaternion<T>, String> {
        let norm_sq = self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z;
        if norm_sq.is_zero() {
            Err("Inverse: Cannot divide by zero norm".to_string())
        } else {
            let conj = self.conjugate();
            Ok(Quaternion {
                w: conj.w / norm_sq,
                x: conj.x / norm_sq,
                y: conj.y / norm_sq,
                z: conj.z / norm_sq,
            })
        }
    }
    pub fn rotate(self, axis: Quaternion<T>, angle: T) -> Result<Quaternion<T>, String> {
        if !axis.w.is_zero() {
            return Err("Rotate: Axis must be a pure quaternion (w=0)".to_string());
        }
        let epsilon = T::from(1e-6).unwrap();
        if !self.is_unit(epsilon) {
            return Err("Rotate: Input quaternion must be a unit quaternion".to_string());
        }

        let axis = axis.normalize().map_err(|e| format!("Rotate -> {}", e))?;
        let half_angle = angle / T::from(2).unwrap();
        let sine = half_angle.sin();
        let rotation_quaternion = Quaternion {
            w: half_angle.cos(),
            x: sine * axis.x,
            y: sine * axis.y,
            z: sine * axis.z,
        };
        Ok((rotation_quaternion * self) * rotation_quaternion.conjugate())
    }
    pub fn nlerp(self, target: Self, t: T) -> Result<Quaternion<T>, String> {
        if t < T::zero() || t > T::one() {
            return Err("NLERP: t must be in [0, 1]".to_string());
        }
        let epsilon = T::from(1e-6).unwrap();
        if !self.is_unit(epsilon) || !target.is_unit(epsilon) {
            return Err("NLERP: Both quaternions must be unit quaternions".to_string());
        }

        let one_minus_t = T::one() - t;
        let result = (self * one_minus_t) + (target * t);
        result.normalize().map_err(|e| format!("NLERP -> {}", e))
    }
    pub fn slerp(self, target: Quaternion<T>, t: T) -> Result<Quaternion<T>, String> {
        if t < T::zero() || t > T::one() {
            return Err("SLERP: t must be in [0, 1]".to_string());
        }
        let epsilon = T::from(1e-6).unwrap();
        if !self.is_unit(epsilon) || !target.is_unit(epsilon) {
            return Err("SLERP: Both quaternions must be unit quaternions".to_string());
        }

        let mut cosine = self.dot(target);
        let mut target = target;
        if cosine < T::zero() {
            cosine = -cosine;
            target = -target;
        }

        if cosine >= T::from(0.9998).unwrap() {
            return self.nlerp(target, t);
        }

        let angle = cosine.acos();
        let sin_angle = angle.sin();
        let one_minus_t_angle = (T::one() - t) * angle;
        let t_angle = t * angle;
        let result = (self * one_minus_t_angle.sin()) + (target * t_angle.sin());

        Ok(result / sin_angle)
    }
    pub fn squad(&self, q1: Quaternion<T>, q2: Quaternion<T>, q3: Quaternion<T>, t: T)
                 -> Result<Quaternion<T>, String> {
        if t < T::zero() || t > T::one() {
            return Err("SQUAD: t must be in [0, 1]".to_string());
        }
        let epsilon = T::from(1e-6).unwrap();
        if !self.is_unit(epsilon) || !q1.is_unit(epsilon) || !q2.is_unit(epsilon) || !q3.is_unit(epsilon) {
            return Err("SQUAD: All quaternions must be unit quaternions".to_string());
        }

        let m1 = self.slerp(q3, t).map_err(|e| format!("SQUAD [1] -> {}", e))?;
        let m2 = q1.slerp(q2, t).map_err(|e| format!("SQUAD [2] -> {}", e))?;
        let final_t = (t + t) * (T::one() - t);

        m1.slerp(m2, final_t).map_err(|e| format!("SQUAD [3] -> {}", e))
    }
}

impl<T> Neg for Quaternion<T>
where
    T: Neg<Output = T> + Copy
{
    type Output = Quaternion<T>;
    #[inline]
    fn neg(self) -> Quaternion<T> {
        Quaternion {
            w: -self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}
