
use bevy::{asset::Assets, core::Name, ecs::{component::Component, entity::Entity, query::Without, system::{Commands, Query, Res, ResMut}}, log::info, math::{primitives::{Capsule3d, Sphere}, Quat, Vec3}, pbr::{PbrBundle, StandardMaterial}, prelude::SpatialBundle, render::{color::Color, mesh::{Mesh, Meshable}}, time::Time, transform::components::Transform};
use bevy_rapier3d::{dynamics::{RigidBody, Velocity}, geometry::{Collider, CollisionGroups, Group}};

use bevy_oxr::xr_input::{hands::{common::{HandBoneRadius, HandResource, HandsResource}, HandBone}, Hand};

pub const FIXED_TIMESTEP: f32 = 1.0 / 60.0;

#[derive(Component, PartialEq, Debug, Clone, Copy)]
pub enum PhysicsHandBone {
    Palm,
    Wrist,
    ThumbMetacarpal,
    ThumbProximal,
    ThumbDistal,
    ThumbTip,
    IndexMetacarpal,
    IndexProximal,
    IndexIntermediate,
    IndexDistal,
    IndexTip,
    MiddleMetacarpal,
    MiddleProximal,
    MiddleIntermediate,
    MiddleDistal,
    MiddleTip,
    RingMetacarpal,
    RingProximal,
    RingIntermediate,
    RingDistal,
    RingTip,
    LittleMetacarpal,
    LittleProximal,
    LittleIntermediate,
    LittleDistal,
    LittleTip,
}

#[derive(Component, PartialEq)]
pub enum BoneInitState {
    True,
    False,
}

pub enum MatchingType {
    PositionMatching,
    VelocityMatching,
}


#[derive(Debug)]
pub struct HandJoints {
    pub inner: [HandJoint; 26],
}

pub enum NameToHandJoint {
    Palm,
    Wrist,
    ThumbMetacarpal,
    ThumbProximal,
    ThumbDistal,
    ThumbTip,
    IndexMetacarpal,
    IndexProximal,
    IndexIntermediate,
    IndexDistal,
    IndexTip,
    MiddleMetacarpal,
    MiddleProximal,
    MiddleIntermediate,
    MiddleDistal,
    MiddleTip,
    RingMetacarpal,
    RingProximal,
    RingIntermediate,
    RingDistal,
    RingTip,
    LittleMetacarpal,
    LittleProximal,
    LittleIntermediate,
    LittleDistal,
    LittleTip,
}



pub fn get_default_right_hand() -> HandJoints {   
    HandJoints {
        inner: [
            // Palm
            HandJoint {
                position: Vec3::new(0.11578956, 1.0322298, -0.07940306),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.16057923, -0.5977889, 0.76988935, -0.15534931),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.017204836,
            },
            // Wrist
            HandJoint {
                position: Vec3::new(0.110605344, 0.96545035, -0.06913033),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.16057923, -0.5977889, 0.76988935, -0.15534931),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.022538071,
            },
            // Thumb Metacarpal - Base of the thumb.
            HandJoint {
                position: Vec3::new(0.13841398, 1.0021406, -0.052384503),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.73630154, -0.30601385, 0.56149095, -0.22123282),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.02035542,
            },
            // Thumb Proximal - First knuckle of the thumb.
            HandJoint {
                position: Vec3::new(0.16202356, 1.0249985, -0.043111823),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.6474834, -0.40439665, 0.59034175, -0.26215592),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.012899493,
            },
            // Thumb Distal - Just before the tip of the thumb.
            HandJoint {
                position: Vec3::new(0.18162939, 1.0539914, -0.03723684),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.75916475, -0.24931243, 0.58079475, -0.15553255),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.010259149,
            },
            // Thumb Tip - Tip of the thumb.
            HandJoint {
                position: Vec3::new(0.20314479, 1.066816, -0.030817013),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.75916475, -0.24931243, 0.58079475, -0.15553255),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.009208953,
            },
            // Index Metacarpal - Base of the index finger.
            HandJoint {
                position: Vec3::new(0.12954025, 1.0039586, -0.061990373),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.16057923, -0.5977889, 0.76988935, -0.15534931),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.022293832,
            },
             // Index Proximal - First knuckle of the index finger.
            HandJoint {
                position: Vec3::new(0.13575831, 1.0662655, -0.0752951),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.17234212, -0.60941154, 0.7517424, -0.18384671),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.010812029,
            },
            // Index Intermediate - Second knuckle of the index finger.
            HandJoint {
                position: Vec3::new(0.13715388, 1.1052845, -0.08317496),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.13962409, -0.6614135, 0.71495426, -0.17854437),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.00896667,
            },
            // Index Distal - Just before the tip of the index finger.
            HandJoint {
                position: Vec3::new(0.13622141, 1.1306962, -0.0853719),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.11572188, -0.6341916, 0.7430719, -0.179595),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.008019494,
            },
            // Index Tip - Tip of the index finger.
            HandJoint {
                position: Vec3::new(0.13507375, 1.1536294, -0.09043077),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.11572188, -0.6341916, 0.7430719, -0.179595),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.006969299,
            },
            // Middle Metacarpal - Base of the middle finger.
            HandJoint {
                position: Vec3::new(0.11431386, 1.0008209, -0.06930854),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.16057923, -0.5977889, 0.76988935, -0.15534931),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.022297699,
            },
            // Middle Proximal - First knuckle of the middle finger.
            HandJoint {
                position: Vec3::new(0.11726526, 1.0636387, -0.08949757),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.10877958, -0.6516118, 0.7275088, -0.18520312),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.011734813,
            },
            // Middle Intermediate - Second knuckle of the middle finger.
            HandJoint {
                position: Vec3::new(0.11351965, 1.1081975, -0.09522918),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.09748417, -0.65516704, 0.7271557, -0.18027195),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.008434071,
            },
            // Middle Distal - Just before the tip of the middle finger.
            HandJoint {
                position: Vec3::new(0.11078716, 1.1367817, -0.09877359),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.07898912, -0.6207319, 0.7657275, -0.14870927),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.008012368,
            },
            // Middle Tip - Tip of the middle finger.
            HandJoint {
                position: Vec3::new(0.109201774, 1.1620579, -0.10566684),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.07898912, -0.6207319, 0.7657275, -0.14870927),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.006962173,
            },
            // Ring Metacarpal - Base of the ring finger.
            HandJoint {
                position: Vec3::new(0.0959552, 1.0016428, -0.07898356),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.16057923, -0.5977889, 0.76988935, -0.15534931),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.02004641,
            },
            // Ring Proximal - First knuckle of the ring finger.
            HandJoint {
                position: Vec3::new(0.0968687, 1.056594, -0.09287319),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.061285853, -0.6445517, 0.7438205, -0.16591744),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.010420178,
            },
            // Ring Intermediate - Second knuckle of the ring finger.
            HandJoint {
                position: Vec3::new(0.09184316, 1.0966957, -0.09949106),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.035476506, -0.65508807, 0.74103206, -0.14308292),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.007993739,
            },
            // Ring Distal - Just before the tip of the ring finger.
            HandJoint {
                position: Vec3::new(0.088078886, 1.1240736, -0.103375815),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.05044055, -0.6750347, 0.7261634, -0.12029514),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.0075940522,
            },
            // Ring Tip - Tip of the ring finger.
            HandJoint {
                position: Vec3::new(0.08594976, 1.1492996, -0.10720982),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.05044055, -0.6750347, 0.7261634, -0.12029514),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.006543857,
            },
            // Little Metacarpal - Base of the little finger.
            HandJoint {
                position: Vec3::new(0.08679972, 1.0013778, -0.07933965),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(0.079209626, -0.6042261, 0.78903735, -0.07782571),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.018996214,
            },
            // Little Proximal - First knuckle of the little finger.
            HandJoint {
                position: Vec3::new(0.076300375, 1.0465008, -0.091672905),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(0.0013647676, -0.611447, 0.7800056, -0.13312519),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.008909174,
            },
            // Little Intermediate - Second knuckle of the little finger.
            HandJoint {
                position: Vec3::new(0.07097942, 1.0772631, -0.09981148),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(0.06511599, -0.6392353, 0.7561073, -0.12425861),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.007103722,
            },
            // Little Distal - Just before the tip of the little finger.
            HandJoint {
                position: Vec3::new(0.065490335, 1.0975376, -0.103528954),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(0.029709637, -0.6588042, 0.746072, -0.09203919),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.006748536,
            },
            // Little Tip - Tip of the little finger.
            HandJoint {
                position: Vec3::new(0.062057115, 1.1199425, -0.1077688),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(0.029709637, -0.6588042, 0.746072, -0.09203919),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.005698341,
            },
        ],
    }
    
}



pub fn get_default_left_hand() -> HandJoints {
    HandJoints {
        inner: [
            HandJoint { position: Vec3::new(-0.09199685, 1.0404115, -0.13863136), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(-0.06417896, -0.61519694, 0.7855916, -0.016115963), orientation_valid: true, orientation_tracked: true, radius: 0.017204836 },
            HandJoint { position: Vec3::new(-0.09377934, 0.9734013, -0.12705012), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(-0.06417896, -0.61519694, 0.7855916, -0.016115963), orientation_valid: true, orientation_tracked: true, radius: 0.022538071 },
            HandJoint { position: Vec3::new(-0.12551022, 1.0117472, -0.12637892), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(0.60534817, -0.3809054, 0.6843805, 0.14173296), orientation_valid: true, orientation_tracked: true, radius: 0.02035542 },
            HandJoint { position: Vec3::new(-0.15011515, 1.0354085, -0.12559117), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(0.515786, -0.46232754, 0.70620716, 0.14659452), orientation_valid: true, orientation_tracked: true, radius: 0.012899493 },
            HandJoint { position: Vec3::new(-0.17115869, 1.0639497, -0.12702624), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(0.62170273, -0.29466128, 0.7214654, 0.078412086), orientation_valid: true, orientation_tracked: true, radius: 0.010259149 },
            HandJoint { position: Vec3::new(-0.19349839, 1.0767481, -0.12942076), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(0.62170273, -0.29466128, 0.7214654, 0.078412086), orientation_valid: true, orientation_tracked: true, radius: 0.009208953 },
            HandJoint { position: Vec3::new(-0.11307493, 1.0127434, -0.130564), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(-0.06417896, -0.61519694, 0.7855916, -0.016115963), orientation_valid: true, orientation_tracked: true, radius: 0.022293832 },
            HandJoint { position: Vec3::new(-0.11093007, 1.0742465, -0.14629754), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(-0.0071815774, -0.7033858, 0.7104458, -0.021536648), orientation_valid: true, orientation_tracked: true, radius: 0.010812029 },
            HandJoint { position: Vec3::new(-0.1117304, 1.1140674, -0.14671154), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(-0.027792111, -0.7521504, 0.6573756, -0.036809683), orientation_valid: true, orientation_tracked: true, radius: 0.00896667 },
            HandJoint { position: Vec3::new(-0.11221108, 1.1393596, -0.1433168), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(-0.057377614, -0.7506119, 0.6576552, -0.02791658), orientation_valid: true, orientation_tracked: true, radius: 0.008019494 },
            HandJoint { position: Vec3::new(-0.110982716, 1.1627451, -0.14120623), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(-0.057377614, -0.7506119, 0.6576552, -0.02791658), orientation_valid: true, orientation_tracked: true, radius: 0.006969299 },
            HandJoint { position: Vec3::new(-0.09627621, 1.0093775, -0.12898207), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(-0.06417896, -0.61519694, 0.7855916, -0.016115963), orientation_valid: true, orientation_tracked: true, radius: 0.022297699 },
            HandJoint { position: Vec3::new(-0.08771748, 1.0714455, -0.14828065), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(-0.13512488, -0.70895374, 0.6920408, 0.014331311), orientation_valid: true, orientation_tracked: true, radius: 0.011734813 },
            HandJoint { position: Vec3::new(-0.07837005, 1.1155072, -0.14639857), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(-0.14643663, -0.7356529, 0.6613316, 0.0034517646), orientation_valid: true, orientation_tracked: true, radius: 0.008434071 },
            HandJoint { position: Vec3::new(-0.072619304, 1.1436299, -0.14277457), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(-0.17750135, -0.71161807, 0.67951804, -0.018677175), orientation_valid: true, orientation_tracked: true, radius: 0.008012368 },
            HandJoint { position: Vec3::new(-0.06635609, 1.169102, -0.14184727), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(-0.17750135, -0.71161807, 0.67951804, -0.018677175), orientation_valid: true, orientation_tracked: true, radius: 0.006962173 },
            HandJoint { position: Vec3::new(-0.075872704, 1.0094653, -0.12763283), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(-0.06417896, -0.61519694, 0.7855916, -0.016115963), orientation_valid: true, orientation_tracked: true, radius: 0.02004641 },
            HandJoint { position: Vec3::new(-0.068767615, 1.0643067, -0.14009632), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(-0.2054858, -0.687606, 0.6962721, 0.013380319), orientation_valid: true, orientation_tracked: true, radius: 0.010420178 },
            HandJoint { position: Vec3::new(-0.05629527, 1.1032953, -0.13886558), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(-0.23049791, -0.71999407, 0.6541984, -0.02244778), orientation_valid: true, orientation_tracked: true, radius: 0.007993739 },
            HandJoint { position: Vec3::new(-0.048781022, 1.1298739, -0.13487369), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(-0.21470001, -0.7524259, 0.6196937, -0.06114711), orientation_valid: true, orientation_tracked: true, radius: 0.0075940522 },
            HandJoint { position: Vec3::new(-0.043416284, 1.1545377, -0.13057244), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(-0.21470001, -0.7524259, 0.6196937, -0.06114711), orientation_valid: true, orientation_tracked: true, radius: 0.006543857 },
            HandJoint { position: Vec3::new(-0.06797077, 1.0091673, -0.12299706), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(-0.29945284, -0.59889627, 0.7368763, -0.09308344), orientation_valid: true, orientation_tracked: true, radius: 0.018996214 },
            HandJoint { position: Vec3::new(-0.052160684, 1.0541556, -0.12795), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(-0.31879357, -0.67544085, 0.6648357, 0.012003437), orientation_valid: true, orientation_tracked: true, radius: 0.008909174 },
            HandJoint { position: Vec3::new(-0.037961796, 1.0828841, -0.12421726), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(-0.37481803, -0.7022579, 0.6051459, -0.011998042), orientation_valid: true, orientation_tracked: true, radius: 0.007103722 },
            HandJoint { position: Vec3::new(-0.02864471, 1.1012058, -0.11851531), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(-0.34324473, -0.7252302, 0.59369886, -0.06120594), orientation_valid: true, orientation_tracked: true, radius: 0.006748536 },
            HandJoint { position: Vec3::new(-0.02077254, 1.1221849, -0.11306969), position_valid: true, position_tracked: true, orientation: Quat::from_xyzw(-0.34324473, -0.7252302, 0.59369886, -0.06120594), orientation_valid: true, orientation_tracked: true, radius: 0.005698341 }
        ]
    }
}


impl NameToHandJoint {
    pub fn get_joint_data(&self, hand: &Hand) -> HandJoint {

        if *hand == Hand::Left {
            return self.get_left_hand_joint_data();
        } else {
            return self.get_right_hand_joint_data();
        }
    }

    fn get_right_hand_joint_data(&self) -> HandJoint {

        match self {
            Self::Palm => HandJoint {
                position: Vec3::new(0.11578956, 1.0322298, -0.07940306),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.16057923, -0.5977889, 0.76988935, -0.15534931),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.017204836,
            },
            Self::Wrist => HandJoint {
                position: Vec3::new(0.110605344, 0.96545035, -0.06913033),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.16057923, -0.5977889, 0.76988935, -0.15534931),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.022538071,
            },
            Self::ThumbMetacarpal => HandJoint {
                position: Vec3::new(0.13841398, 1.0021406, -0.052384503),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.73630154, -0.30601385, 0.56149095, -0.22123282),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.02035542,
            },
            Self::ThumbProximal => HandJoint {
                position: Vec3::new(0.16202356, 1.0249985, -0.043111823),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.6474834, -0.40439665, 0.59034175, -0.26215592),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.012899493,
            },
            Self::ThumbDistal => HandJoint {
                position: Vec3::new(0.18162939, 1.0539914, -0.03723684),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.75916475, -0.24931243, 0.58079475, -0.15553255),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.010259149,
            },
            Self::ThumbTip => HandJoint {
                position: Vec3::new(0.20314479, 1.066816, -0.030817013),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.75916475, -0.24931243, 0.58079475, -0.15553255),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.009208953,
            },
            Self::IndexMetacarpal => HandJoint {
                position: Vec3::new(0.12954025, 1.0039586, -0.061990373),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.16057923, -0.5977889, 0.76988935, -0.15534931),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.022293832,
            },
            Self::IndexProximal => HandJoint {
                position: Vec3::new(0.13575831, 1.0662655, -0.0752951),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.17234212, -0.60941154, 0.7517424, -0.18384671),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.010812029,
            },
            Self::IndexIntermediate => HandJoint {
                position: Vec3::new(0.13715388, 1.1052845, -0.08317496),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.13962409, -0.6614135, 0.71495426, -0.17854437),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.00896667,
            },
            Self::IndexDistal => HandJoint {
                position: Vec3::new(0.13622141, 1.1306962, -0.0853719),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.11572188, -0.6341916, 0.7430719, -0.179595),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.008019494,
            },
            Self::IndexTip => HandJoint {
                position: Vec3::new(0.13507375, 1.1536294, -0.09043077),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.11572188, -0.6341916, 0.7430719, -0.179595),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.006969299,
            },
            Self::MiddleMetacarpal => HandJoint {
                position: Vec3::new(0.11431386, 1.0008209, -0.06930854),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.16057923, -0.5977889, 0.76988935, -0.15534931),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.022297699,
            },
            Self::MiddleProximal => HandJoint {
                position: Vec3::new(0.11726526, 1.0636387, -0.08949757),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.10877958, -0.6516118, 0.7275088, -0.18520312),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.011734813,
            },
            Self::MiddleIntermediate => HandJoint {
                position: Vec3::new(0.11351965, 1.1081975, -0.09522918),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.09748417, -0.65516704, 0.7271557, -0.18027195),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.008434071,
            },
            Self::MiddleDistal => HandJoint {
                position: Vec3::new(0.11078716, 1.1367817, -0.09877359),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.07898912, -0.6207319, 0.7657275, -0.14870927),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.008012368,
            },
            Self::MiddleTip => HandJoint {
                position: Vec3::new(0.109201774, 1.1620579, -0.10566684),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.07898912, -0.6207319, 0.7657275, -0.14870927),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.006962173,
            },
            Self::RingMetacarpal => HandJoint {
                position: Vec3::new(0.0959552, 1.0016428, -0.07898356),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.16057923, -0.5977889, 0.76988935, -0.15534931),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.02004641,
            },
            Self::RingProximal => HandJoint {
                position: Vec3::new(0.0968687, 1.056594, -0.09287319),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.061285853, -0.6445517, 0.7438205, -0.16591744),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.010420178,
            },
            Self::RingIntermediate => HandJoint {
                position: Vec3::new(0.09184316, 1.0966957, -0.09949106),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.035476506, -0.65508807, 0.74103206, -0.14308292),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.007993739,
            },
            Self::RingDistal => HandJoint {
                position: Vec3::new(0.088078886, 1.1240736, -0.103375815),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.05044055, -0.6750347, 0.7261634, -0.12029514),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.0075940522,
            },
            Self::RingTip => HandJoint {
                position: Vec3::new(0.08594976, 1.1492996, -0.10720982),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(-0.05044055, -0.6750347, 0.7261634, -0.12029514),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.006543857,
            },
            Self::LittleMetacarpal => HandJoint {
                position: Vec3::new(0.08679972, 1.0013778, -0.07933965),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(0.079209626, -0.6042261, 0.78903735, -0.07782571),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.018996214,
            },
            Self::LittleProximal => HandJoint {
                position: Vec3::new(0.076300375, 1.0465008, -0.091672905),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(0.0013647676, -0.611447, 0.7800056, -0.13312519),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.008909174,
            },
            Self::LittleIntermediate => HandJoint {
                position: Vec3::new(0.07097942, 1.0772631, -0.09981148),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(0.06511599, -0.6392353, 0.7561073, -0.12425861),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.007103722,
            },
            Self::LittleDistal => HandJoint {
                position: Vec3::new(0.065490335, 1.0975376, -0.103528954),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(0.029709637, -0.6588042, 0.746072, -0.09203919),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.006748536,
            },
            Self::LittleTip => HandJoint {
                position: Vec3::new(0.062057115, 1.1199425, -0.1077688),
                position_valid: true,
                position_tracked: true,
                orientation: Quat::from_xyzw(0.029709637, -0.6588042, 0.746072, -0.09203919),
                orientation_valid: true,
                orientation_tracked: true,
                radius: 0.005698341,
            },
        }
    }

    fn get_left_hand_joint_data(&self) -> HandJoint {

        let hand_joints = get_default_left_hand();

        match self {
            NameToHandJoint::Palm => hand_joints.inner[0],
            NameToHandJoint::Wrist => hand_joints.inner[1],
            NameToHandJoint::ThumbMetacarpal => hand_joints.inner[2],
            NameToHandJoint::ThumbProximal => hand_joints.inner[3],
            NameToHandJoint::ThumbDistal => hand_joints.inner[4],
            NameToHandJoint::ThumbTip => hand_joints.inner[5],
            NameToHandJoint::IndexMetacarpal => hand_joints.inner[6],
            NameToHandJoint::IndexProximal => hand_joints.inner[7],
            NameToHandJoint::IndexIntermediate => hand_joints.inner[8],
            NameToHandJoint::IndexDistal => hand_joints.inner[9],
            NameToHandJoint::IndexTip => hand_joints.inner[10],
            NameToHandJoint::MiddleMetacarpal => hand_joints.inner[11],
            NameToHandJoint::MiddleProximal => hand_joints.inner[12],
            NameToHandJoint::MiddleIntermediate => hand_joints.inner[13],
            NameToHandJoint::MiddleDistal => hand_joints.inner[14],
            NameToHandJoint::MiddleTip => hand_joints.inner[15],
            NameToHandJoint::RingMetacarpal => hand_joints.inner[16],
            NameToHandJoint::RingProximal => hand_joints.inner[17],
            NameToHandJoint::RingIntermediate => hand_joints.inner[18],
            NameToHandJoint::RingDistal => hand_joints.inner[19],
            NameToHandJoint::RingTip => hand_joints.inner[20],
            NameToHandJoint::LittleMetacarpal => hand_joints.inner[21],
            NameToHandJoint::LittleProximal => hand_joints.inner[22],
            NameToHandJoint::LittleIntermediate => hand_joints.inner[23],
            NameToHandJoint::LittleDistal => hand_joints.inner[24],
            NameToHandJoint::LittleTip => hand_joints.inner[25],
        }
    }
    pub fn get_physics_bone_from_index(index: usize) -> PhysicsHandBone {
        match index {
            0 => PhysicsHandBone::Palm,
            1 => PhysicsHandBone::Wrist,
            2 => PhysicsHandBone::ThumbMetacarpal,
            3 => PhysicsHandBone::ThumbProximal,
            4 => PhysicsHandBone::ThumbDistal,
            5 => PhysicsHandBone::ThumbTip,
            6 => PhysicsHandBone::IndexMetacarpal,
            7 => PhysicsHandBone::IndexProximal,
            8 => PhysicsHandBone::IndexIntermediate,
            9 => PhysicsHandBone::IndexDistal,
            10 => PhysicsHandBone::IndexTip,
            11 => PhysicsHandBone::MiddleMetacarpal,
            12 => PhysicsHandBone::MiddleProximal,
            13 => PhysicsHandBone::MiddleIntermediate,
            14 => PhysicsHandBone::MiddleDistal,
            15 => PhysicsHandBone::MiddleTip,
            16 => PhysicsHandBone::RingMetacarpal,
            17 => PhysicsHandBone::RingProximal,
            18 => PhysicsHandBone::RingIntermediate,
            19 => PhysicsHandBone::RingDistal,
            20 => PhysicsHandBone::RingTip,
            21 => PhysicsHandBone::LittleMetacarpal,
            22 => PhysicsHandBone::LittleProximal,
            23 => PhysicsHandBone::LittleIntermediate,
            24 => PhysicsHandBone::LittleDistal,
            25 => PhysicsHandBone::LittleTip,
            _ => panic!("Index out of bounds"),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct HandJoint {
    pub position: Vec3,
    pub position_valid: bool,
    pub position_tracked: bool,
    pub orientation: Quat,
    pub orientation_valid: bool,
    pub orientation_tracked: bool,
    pub radius: f32,
}





pub fn get_start_and_end_joints(
    bone: &PhysicsHandBone,
    hand: &Hand,
) -> Option<(HandJoint, HandJoint)> {

    

    match bone {
        PhysicsHandBone::Palm => return None,
        PhysicsHandBone::Wrist => return None,
        PhysicsHandBone::ThumbMetacarpal => {
            let joint_one = NameToHandJoint::ThumbMetacarpal.get_joint_data(hand);
            let joint_two = NameToHandJoint::ThumbProximal.get_joint_data(hand);
            return Some((joint_one, joint_two));
        },
        PhysicsHandBone::ThumbProximal => {
            let joint_one = NameToHandJoint::ThumbProximal.get_joint_data(hand);
            let joint_two = NameToHandJoint::ThumbDistal.get_joint_data(hand);
            return Some((joint_one, joint_two));
        },
        PhysicsHandBone::ThumbDistal => {
            let joint_one = NameToHandJoint::ThumbDistal.get_joint_data(hand);
            let joint_two = NameToHandJoint::ThumbTip.get_joint_data(hand);
            return Some((joint_one, joint_two));
        },
        PhysicsHandBone::ThumbTip => return None,
        PhysicsHandBone::IndexMetacarpal => {
            let joint_one = NameToHandJoint::IndexMetacarpal.get_joint_data(hand);
            let joint_two = NameToHandJoint::IndexProximal.get_joint_data(hand);
            return Some((joint_one, joint_two));
        },
        PhysicsHandBone::IndexProximal => {
            let joint_one = NameToHandJoint::IndexProximal.get_joint_data(hand);
            let joint_two = NameToHandJoint::IndexIntermediate.get_joint_data(hand);
            return Some((joint_one, joint_two));
        },
        PhysicsHandBone::IndexIntermediate => {
            let joint_one = NameToHandJoint::IndexIntermediate.get_joint_data(hand);
            let joint_two = NameToHandJoint::IndexDistal.get_joint_data(hand);
            return Some((joint_one, joint_two));
        },
        PhysicsHandBone::IndexDistal => {
            let joint_one = NameToHandJoint::IndexDistal.get_joint_data(hand);
            let joint_two = NameToHandJoint::IndexTip.get_joint_data(hand);
            return Some((joint_one, joint_two));
        },
        PhysicsHandBone::IndexTip => return None,
        PhysicsHandBone::MiddleMetacarpal => {
            let joint_one = NameToHandJoint::MiddleMetacarpal.get_joint_data(hand);
            let joint_two = NameToHandJoint::MiddleProximal.get_joint_data(hand);
            return Some((joint_one, joint_two));
        },
        PhysicsHandBone::MiddleProximal => {
            let joint_one = NameToHandJoint::MiddleProximal.get_joint_data(hand);
            let joint_two = NameToHandJoint::MiddleIntermediate.get_joint_data(hand);
            return Some((joint_one, joint_two));
        },
        PhysicsHandBone::MiddleIntermediate => {
            let joint_one = NameToHandJoint::MiddleIntermediate.get_joint_data(hand);
            let joint_two = NameToHandJoint::MiddleDistal.get_joint_data(hand);
            return Some((joint_one, joint_two));
        },
        PhysicsHandBone::MiddleDistal => {
            let joint_one = NameToHandJoint::MiddleDistal.get_joint_data(hand);
            let joint_two = NameToHandJoint::MiddleTip.get_joint_data(hand);
            return Some((joint_one, joint_two));
        },
        PhysicsHandBone::MiddleTip => return None,
        PhysicsHandBone::RingMetacarpal => {
            let joint_one = NameToHandJoint::RingMetacarpal.get_joint_data(hand);
            let joint_two = NameToHandJoint::RingProximal.get_joint_data(hand);
            return Some((joint_one, joint_two));
        },
        PhysicsHandBone::RingProximal => {
            let joint_one = NameToHandJoint::RingProximal.get_joint_data(hand);
            let joint_two = NameToHandJoint::RingIntermediate.get_joint_data(hand);
            return Some((joint_one, joint_two));
        },
        PhysicsHandBone::RingIntermediate => {
            let joint_one = NameToHandJoint::RingIntermediate.get_joint_data(hand);
            let joint_two = NameToHandJoint::RingDistal.get_joint_data(hand);
            return Some((joint_one, joint_two));
        },
        PhysicsHandBone::RingDistal => {
            let joint_one = NameToHandJoint::RingDistal.get_joint_data(hand);
            let joint_two = NameToHandJoint::RingTip.get_joint_data(hand);
            return Some((joint_one, joint_two));
        },
        PhysicsHandBone::RingTip => return None,
        PhysicsHandBone::LittleMetacarpal => {
            let joint_one = NameToHandJoint::LittleMetacarpal.get_joint_data(hand);
            let joint_two = NameToHandJoint::LittleProximal.get_joint_data(hand);
            return Some((joint_one, joint_two));
        },
        PhysicsHandBone::LittleProximal => {
            let joint_one = NameToHandJoint::LittleProximal.get_joint_data(hand);
            let joint_two = NameToHandJoint::LittleIntermediate.get_joint_data(hand);
            return Some((joint_one, joint_two));
        },
        PhysicsHandBone::LittleIntermediate => {
            let joint_one = NameToHandJoint::LittleIntermediate.get_joint_data(hand);
            let joint_two = NameToHandJoint::LittleDistal.get_joint_data(hand);
            return Some((joint_one, joint_two));
        },
        PhysicsHandBone::LittleDistal => {
            let joint_one = NameToHandJoint::LittleDistal.get_joint_data(hand);
            let joint_two = NameToHandJoint::LittleTip.get_joint_data(hand);
            return Some((joint_one, joint_two));
        },
        PhysicsHandBone::LittleTip => return None,
    }

}


pub fn get_start_and_end_entities(
    hand_res: HandResource,
    bone: &PhysicsHandBone,
) -> Option<(Entity, Entity)> {
    match bone {
        PhysicsHandBone::Palm => return None,
        PhysicsHandBone::Wrist => return None,
        PhysicsHandBone::ThumbMetacarpal => {
            return Some((hand_res.thumb.metacarpal, hand_res.thumb.proximal))
        }
        PhysicsHandBone::ThumbProximal => {
            return Some((hand_res.thumb.proximal, hand_res.thumb.distal))
        }
        PhysicsHandBone::ThumbDistal => return Some((hand_res.thumb.distal, hand_res.thumb.tip)),
        PhysicsHandBone::ThumbTip => return None,
        PhysicsHandBone::IndexMetacarpal => {
            return Some((hand_res.index.metacarpal, hand_res.index.proximal))
        }
        PhysicsHandBone::IndexProximal => {
            return Some((hand_res.index.proximal, hand_res.index.intermediate))
        }
        PhysicsHandBone::IndexIntermediate => {
            return Some((hand_res.index.intermediate, hand_res.index.distal))
        }
        PhysicsHandBone::IndexDistal => return Some((hand_res.index.distal, hand_res.index.tip)),
        PhysicsHandBone::IndexTip => return None,
        PhysicsHandBone::MiddleMetacarpal => {
            return Some((hand_res.middle.metacarpal, hand_res.middle.proximal))
        }
        PhysicsHandBone::MiddleProximal => {
            return Some((hand_res.middle.proximal, hand_res.middle.intermediate))
        }
        PhysicsHandBone::MiddleIntermediate => {
            return Some((hand_res.middle.intermediate, hand_res.middle.distal))
        }
        PhysicsHandBone::MiddleDistal => {
            return Some((hand_res.middle.distal, hand_res.middle.tip))
        }
        PhysicsHandBone::MiddleTip => return None,
        PhysicsHandBone::RingMetacarpal => {
            return Some((hand_res.ring.metacarpal, hand_res.ring.proximal))
        }
        PhysicsHandBone::RingProximal => {
            return Some((hand_res.ring.proximal, hand_res.ring.intermediate))
        }
        PhysicsHandBone::RingIntermediate => {
            return Some((hand_res.ring.intermediate, hand_res.ring.distal))
        }
        PhysicsHandBone::RingDistal => return Some((hand_res.ring.distal, hand_res.ring.tip)),
        PhysicsHandBone::RingTip => return None,
        PhysicsHandBone::LittleMetacarpal => {
            return Some((hand_res.little.metacarpal, hand_res.little.proximal))
        }
        PhysicsHandBone::LittleProximal => {
            return Some((hand_res.little.proximal, hand_res.little.intermediate))
        }
        PhysicsHandBone::LittleIntermediate => {
            return Some((hand_res.little.intermediate, hand_res.little.distal))
        }
        PhysicsHandBone::LittleDistal => {
            return Some((hand_res.little.distal, hand_res.little.tip))
        }
        PhysicsHandBone::LittleTip => return None,
    };
}


pub fn spawn_hand_entities(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let hands = [Hand::Left, Hand::Right];
    let bones = HandBone::get_all_bones();

    // let right_hand = get_default_right_hand();
    // let left_hand = get_default_left_hand();

    // let hand_joints = get_default_right_hand();
    //hand resource
    let mut hand_resource = HandsResource { ..Default::default() };
    for hand in hands.iter() {

        for bone in bones.iter() {

            let physics_bone_index = bone.get_index_from_bone();

            let physics_bone = NameToHandJoint::get_physics_bone_from_index(physics_bone_index);

            let joints_opt = get_start_and_end_joints(&physics_bone, &hand);

            if joints_opt.is_none() {
                continue;
            }

            let (joint_one, joint_two) = joints_opt.unwrap();


            let direction = joint_two.position - joint_one.position;

            let length = direction.length();

            let orientation = joint_one.orientation;


            let boneid = commands
                .spawn((
                    Name::new(format!("{:?} {:?}", hand, bone)),
                    // SpatialBundle::default(),
                    PbrBundle {
                        mesh: meshes.add(Sphere::new(joint_one.radius)),
                        material: materials.add(Color::rgb(0.8, 0.7, 0.6)),
                        transform: Transform {
                            translation: direction,
                            rotation: orientation,
                            ..Default::default()
                        },
                        ..Default::default()
                    },
                    *bone,
                    *hand,
                    HandBoneRadius(0.1),
                ))
                .id();
            let hand_res = match hand {
                Hand::Left => &mut hand_resource.left,
                Hand::Right => &mut hand_resource.right,
            };

            match bone {
                HandBone::Palm => hand_res.palm = boneid,
                HandBone::Wrist => hand_res.wrist = boneid,
                HandBone::ThumbMetacarpal => hand_res.thumb.metacarpal = boneid,
                HandBone::ThumbProximal => hand_res.thumb.proximal = boneid,
                HandBone::ThumbDistal => hand_res.thumb.distal = boneid,
                HandBone::ThumbTip => hand_res.thumb.tip = boneid,
                HandBone::IndexMetacarpal => hand_res.index.metacarpal = boneid,
                HandBone::IndexProximal => hand_res.index.proximal = boneid,
                HandBone::IndexIntermediate => hand_res.index.intermediate = boneid,
                HandBone::IndexDistal => hand_res.index.distal = boneid,
                HandBone::IndexTip => hand_res.index.tip = boneid,
                HandBone::MiddleMetacarpal => hand_res.middle.metacarpal = boneid,
                HandBone::MiddleProximal => hand_res.middle.proximal = boneid,
                HandBone::MiddleIntermediate => hand_res.middle.intermediate = boneid,
                HandBone::MiddleDistal => hand_res.middle.distal = boneid,
                HandBone::MiddleTip => hand_res.middle.tip = boneid,
                HandBone::RingMetacarpal => hand_res.ring.metacarpal = boneid,
                HandBone::RingProximal => hand_res.ring.proximal = boneid,
                HandBone::RingIntermediate => hand_res.ring.intermediate = boneid,
                HandBone::RingDistal => hand_res.ring.distal = boneid,
                HandBone::RingTip => hand_res.ring.tip = boneid,
                HandBone::LittleMetacarpal => hand_res.little.metacarpal = boneid,
                HandBone::LittleProximal => hand_res.little.proximal = boneid,
                HandBone::LittleIntermediate => hand_res.little.intermediate = boneid,
                HandBone::LittleDistal => hand_res.little.distal = boneid,
                HandBone::LittleTip => hand_res.little.tip = boneid,
            }
        }
    
        
    }

    commands.insert_resource(hand_resource);
}




pub fn spawn_physics_hands(
    mut commands: Commands,
    hands_res: Res<HandsResource>,
    hand_query: Query<(&Transform, &HandBone, &Hand), Without<PhysicsHandBone>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>

) {
    let hands = [Hand::Left, Hand::Right];
    let bones = [
        PhysicsHandBone::Palm,
        PhysicsHandBone::Wrist,
        PhysicsHandBone::ThumbMetacarpal,
        PhysicsHandBone::ThumbProximal,
        PhysicsHandBone::ThumbDistal,
        PhysicsHandBone::ThumbTip,
        PhysicsHandBone::IndexMetacarpal,
        PhysicsHandBone::IndexProximal,
        PhysicsHandBone::IndexIntermediate,
        PhysicsHandBone::IndexDistal,
        PhysicsHandBone::IndexTip,
        PhysicsHandBone::MiddleMetacarpal,
        PhysicsHandBone::MiddleProximal,
        PhysicsHandBone::MiddleIntermediate,
        PhysicsHandBone::MiddleDistal,
        PhysicsHandBone::MiddleTip,
        PhysicsHandBone::RingMetacarpal,
        PhysicsHandBone::RingProximal,
        PhysicsHandBone::RingIntermediate,
        PhysicsHandBone::RingDistal,
        PhysicsHandBone::RingTip,
        PhysicsHandBone::LittleMetacarpal,
        PhysicsHandBone::LittleProximal,
        PhysicsHandBone::LittleIntermediate,
        PhysicsHandBone::LittleDistal,
        PhysicsHandBone::LittleTip,
    ];
    let radius = 0.010;
    let left_hand_membership_group = Group::GROUP_1;
    let right_hand_membership_group = Group::GROUP_2;
    let floor_membership = Group::GROUP_3;

    // let hand_joints = get_default_right_hand();

    println!("hand_query {:?}", hand_query);

    for hand in hands.iter() {

        let hand_joints = match hand {
            Hand::Left => get_default_left_hand(),
            Hand::Right => get_default_right_hand(),
        };

        let hand_membership = match hand {
            Hand::Left => left_hand_membership_group,
            Hand::Right => right_hand_membership_group,
        };
        let mut hand_filter: Group = Group::ALL;
        hand_filter.remove(hand_membership);
        hand_filter.remove(floor_membership);

        // Collider::compound()

        for joint in hand_joints.inner.iter() {


            let color = match hand {
                Hand::Left => Color::rgb(0.8, 0.7, 0.6),
                Hand::Right => Color::rgb(0.6, 0.7, 0.8),
            };

            //spawn the thing
            commands.spawn((
                // SpatialBundle::default(),
                PbrBundle {
                    mesh: meshes.add(Sphere::new(joint.radius)),
                    material: materials.add(color),
                    transform: Transform {
                        translation: joint.position,
                        rotation: joint.orientation,
                        ..Default::default()
                    },
                    ..Default::default()
                },
                Collider::capsule(
                    Vec3 {
                        x: 0.0,
                        y: -0.0575,
                        z: 0.0,
                    },
                    Vec3 {
                        x: 0.0,
                        y: 0.0575,
                        z: 0.0,
                    },
                    joint.radius / 2.0,
                ),
                RigidBody::Fixed,
                Velocity::default(),
                CollisionGroups::new(hand_membership, hand_filter),
                // SolverGroups::new(self_group, interaction_group),
                PhysicsHandBone::Palm,
                *hand,
                
            ));
        }

        // for bone in bones.iter() {

        //     if Some(hands_res.clone()).is_none() {
        //         info!("hands resource not initialized yet");
        //         return;
        //     }

            

        //     let joints_opt: Option<(Entity, Entity)> = get_start_and_end_entities(hands_res.right, bone);

        //     if joints_opt.is_none() {
        //         info!("joints is none");
        //         continue;
        //     }

        //     let joints =  joints_opt.unwrap();

        //     let start_components = hand_query.get(joints.0);

        //     if !start_components.is_ok() {
        //         info!("start components are none");
        //         continue;
        //     }

        //     let end_components = hand_query.get(joints.1);

        //     if !end_components.is_ok() {
        //         info!("end components are none");
        //         continue;
        //     }

        //     println!("start_components {:?}", start_components);
        //     println!("end_components {:?}", end_components);

        //     let direction = end_components.unwrap().0.translation - start_components.unwrap().0.translation;

        //     // println!("start_components {:?}", start_components);
        //     // println!("end_components {:?}", end_components);
        //     println!("direction {:?}", direction);

        //     let orientation = start_components.unwrap().0.rotation;

        //     println!("orientation {:?}", orientation);

        //     if direction.length() < 0.001 {
        //         info!("direction length is zero");
        //         continue;
        //     }

            
        //     // let joint = hand_joints.inner.iter().find(|&x| x.position == joint_position).unwrap();



        //     //spawn the thing
        //     commands.spawn((
        //         // SpatialBundle::default(),
        //         // SpatialBundle {
        //         //     transform: Default::default(),
        //         //     ..Default::default()
        //         // },
        //         PbrBundle {
        //             mesh: meshes.add(Capsule3d::new(
        //                 0.1,
        //                 direction.length(),
        //             )),
        //             material: materials.add(Color::rgb(0.0, 0.0, 0.0)),
        //             transform: Transform {
        //                 translation: direction,
        //                 rotation: orientation,
        //                 ..Default::default()
        //             },
        //             ..Default::default()
        //         },
        //         Collider::capsule(
        //             start_components.unwrap().0.translation, 
        //             end_components.unwrap().0.translation, 
        //             radius
        //         ),
        //         // Collider::capsule(
        //         //     Vec3 {
        //         //         x: 0.0,
        //         //         y: -0.0575,
        //         //         z: 0.0,
        //         //     },
        //         //     Vec3 {
        //         //         x: 0.0,
        //         //         y: 0.0575,
        //         //         z: 0.0,
        //         //     },
        //         //     radius,
        //         // ),
        //         RigidBody::Fixed,
        //         Velocity::default(),
        //         CollisionGroups::new(hand_membership, Group::from_bits(0b0001).unwrap()),
        //         // SolverGroups::new(self_group, interaction_group),
        //         BoneInitState::False,
        //         bone.clone(),
        //         hand.clone(),
        //     ));
        // }
    
    }

}


pub fn update_physics_hands(
    hands_res: Option<Res<HandsResource>>,
    mut bone_query: Query<(
        &mut Transform,
        &mut Collider,
        &PhysicsHandBone,
        &mut BoneInitState,
        &Hand,
        &mut Velocity,
    )>,
    hand_query: Query<(&Transform, &HandBone, &Hand), Without<PhysicsHandBone>>,
    time: Res<Time>,
) {

    let matching = MatchingType::VelocityMatching;
    //sanity check do we even have hands?
    match hands_res {
        Some(res) => {

            //config stuff
            let radius = 0.010;
            for mut bone in bone_query.iter_mut() {

                if *bone.4 == Hand::Left {
                    continue;
                }

                let hand_res = match bone.4 {
                    Hand::Left => res.left,
                    Hand::Right => res.right,
                };

                //lets just do the Right ThumbMetacarpal for now
                let result = get_start_and_end_entities(hand_res, bone.2);
                if let Some((start_entity, end_entity)) = result {
                    //now we need their transforms
                    let start_components = hand_query.get(start_entity);
                    let end_components = hand_query.get(end_entity);
                    let direction = end_components.unwrap().0.translation
                        - start_components.unwrap().0.translation;
                    if direction.length() < 0.001 {
                        //i hate this but we need to skip init if the length is zero
                        return;
                    }

                    match *bone.3 {
                        BoneInitState::True => {
                            match matching {
                                MatchingType::PositionMatching => {
                                    //if we are init then we just move em?
                                    *bone.0 = start_components
                                        .unwrap()
                                        .0
                                        .clone()
                                        .looking_at(end_components.unwrap().0.translation, Vec3::Y);
                                }
                                MatchingType::VelocityMatching => {
                                    //calculate position difference
                                    let diff = (start_components.unwrap().0.translation
                                        - bone.0.translation)
                                        / time.delta_seconds();
                                    bone.5.linvel = diff;
                                    //calculate angular velocity?
                                    // gizmos.ray(bone.0.translation, bone.0.forward(), Color::WHITE);
                                    let desired_forward = start_components
                                        .unwrap()
                                        .0
                                        .clone()
                                        .looking_at(end_components.unwrap().0.translation, Vec3::Y)
                                        .rotation;
                                    // gizmos.ray(
                                    //     bone.0.translation,
                                    //     desired_forward.mul_vec3(-Vec3::Z),
                                    //     Color::GREEN,
                                    // );
                                    let cross =
                                        bone.0.forward().cross(desired_forward.mul_vec3(-Vec3::Z));

                                    // gizmos.ray(
                                    //     bone.0.translation,
                                    //     cross,
                                    //     Color::RED,
                                    // );
                                    bone.5.angvel = cross / time.delta_seconds();
                                }
                            }
                        }
                        BoneInitState::False => {
                            //build a new collider?
                            *bone.1 = Collider::capsule(
                                Vec3::splat(0.0),
                                Vec3 {
                                    x: 0.0,
                                    y: 0.0,
                                    z: -direction.length(),
                                },
                                radius,
                            );
                            *bone.3 = BoneInitState::True;
                        }
                    }
                }
            }
        }
        None => info!("hand states resource not initialized yet"),
    }

}