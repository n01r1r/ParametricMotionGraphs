#include "pmg/BvhLoader.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace pmg {

namespace {

BvhChannelType ParseChannelType(const std::string& token) {
    if (token == "Xposition") return BvhChannelType::XPosition;
    if (token == "Yposition") return BvhChannelType::YPosition;
    if (token == "Zposition") return BvhChannelType::ZPosition;
    if (token == "Xrotation") return BvhChannelType::XRotation;
    if (token == "Yrotation") return BvhChannelType::YRotation;
    if (token == "Zrotation") return BvhChannelType::ZRotation;
    throw std::runtime_error("BvhLoader: unsupported channel type: " + token);
}

std::vector<std::string> ReadTokens(const std::string& path) {
    std::ifstream stream(path);
    if (!stream) {
        throw std::runtime_error("BvhLoader: failed to open file: " + path);
    }

    std::vector<std::string> tokens;
    std::string token;
    while (stream >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

class TokenCursor {
public:
    explicit TokenCursor(std::vector<std::string> tokens)
        : tokens_(std::move(tokens)) {}

    bool HasNext() const {
        return index_ < tokens_.size();
    }

    const std::string& Peek() const {
        if (!HasNext()) {
            throw std::runtime_error("BvhLoader: unexpected end of file");
        }
        return tokens_[index_];
    }

    std::string Next() {
        const std::string token = Peek();
        ++index_;
        return token;
    }

    void Expect(const std::string& expected) {
        const std::string actual = Next();
        if (actual != expected) {
            throw std::runtime_error(
                "BvhLoader: expected '" + expected + "' but got '" + actual + "'");
        }
    }

    int NextInt() {
        return std::stoi(Next());
    }

    float NextFloat() {
        return std::stof(Next());
    }

private:
    std::vector<std::string> tokens_;
    std::size_t index_ = 0;
};

Vec3 ReadBvhVector(TokenCursor& cursor) {
    // C++ does not guarantee left-to-right evaluation of repeated function
    // arguments. Read BVH x/y/z values into named variables before constructing
    // Vec3 so MSVC/Clang/GCC all preserve the file order.
    // Values are kept in the file's native units: any display scaling is a
    // render-time concern (the viewer applies it), so the core geometry, FK and
    // point-cloud metric all operate at the BVH's own scale.
    const float x = cursor.NextFloat();
    const float y = cursor.NextFloat();
    const float z = cursor.NextFloat();

    return Vec3(x, y, z);
}

void ParseJointBlock(
    TokenCursor& cursor,
    Skeleton& skeleton,
    const std::string& joint_name,
    int parent_index) {
    const int joint_index = skeleton.NumJoints();
    Joint joint;
    joint.name = joint_name;
    joint.parent_index = parent_index;
    skeleton.joints.push_back(joint);

    cursor.Expect("{");

    while (cursor.HasNext()) {
        const std::string token = cursor.Next();

        if (token == "}") {
            return;
        }

        if (token == "OFFSET") {
            skeleton.joints[joint_index].offset = ReadBvhVector(cursor);
            continue;
        }

        if (token == "CHANNELS") {
            const int channel_count = cursor.NextInt();
            if (channel_count <= 0) {
                throw std::runtime_error("BvhLoader: CHANNELS count must be positive");
            }
            skeleton.joints[joint_index].channels.reserve(channel_count);
            for (int channel_index = 0; channel_index < channel_count; ++channel_index) {
                skeleton.joints[joint_index].channels.push_back(ParseChannelType(cursor.Next()));
            }
            continue;
        }

        if (token == "JOINT") {
            const std::string child_name = cursor.Next();
            ParseJointBlock(cursor, skeleton, child_name, joint_index);
            continue;
        }

        if (token == "End") {
            // `End` consumed; format is `End Site [optional name] { OFFSET x y z }`.
            // Store the leaf as a zero-channel joint so its tip renders and feeds
            // the point-cloud metric; it carries no motion channels.
            cursor.Expect("Site");

            std::string end_site_name = joint_name + "_End";
            if (cursor.Peek() != "{") {
                end_site_name = cursor.Next();  // named End Site variant
            }
            cursor.Expect("{");

            Joint end_site_joint;
            end_site_joint.name = end_site_name;
            end_site_joint.parent_index = joint_index;

            while (cursor.HasNext()) {
                const std::string end_token = cursor.Next();
                if (end_token == "}") {
                    break;
                }
                if (end_token == "OFFSET") {
                    end_site_joint.offset = ReadBvhVector(cursor);
                    continue;
                }
                throw std::runtime_error("BvhLoader: unsupported End Site token: " + end_token);
            }

            skeleton.joints.push_back(end_site_joint);
            continue;
        }

        throw std::runtime_error("BvhLoader: unsupported joint token: " + token);
    }

    throw std::runtime_error("BvhLoader: unterminated joint block");
}

int TotalChannelCount(const Skeleton& skeleton) {
    int total = 0;
    for (const Joint& joint : skeleton.joints) {
        total += static_cast<int>(joint.channels.size());
    }
    return total;
}

Pose ParsePoseFrame(TokenCursor& cursor, const Skeleton& skeleton) {
    Pose pose;
    pose.local_rotations.assign(static_cast<std::size_t>(skeleton.NumJoints()), Quaternion::Identity());

    for (int joint_index = 0; joint_index < skeleton.NumJoints(); ++joint_index) {
        const Joint& joint = skeleton.joints[joint_index];
        Quaternion local_rotation = Quaternion::Identity();

        for (BvhChannelType channel : joint.channels) {
            const float value = cursor.NextFloat();

            switch (channel) {
                case BvhChannelType::XPosition:
                    if (joint.parent_index < 0) {
                        pose.root_position.x = value;
                    }
                    break;
                case BvhChannelType::YPosition:
                    if (joint.parent_index < 0) {
                        pose.root_position.y = value;
                    }
                    break;
                case BvhChannelType::ZPosition:
                    if (joint.parent_index < 0) {
                        pose.root_position.z = value;
                    }
                    break;
                case BvhChannelType::XRotation:
                    local_rotation = local_rotation * EulerAxisRotation('X', value);
                    break;
                case BvhChannelType::YRotation:
                    local_rotation = local_rotation * EulerAxisRotation('Y', value);
                    break;
                case BvhChannelType::ZRotation:
                    local_rotation = local_rotation * EulerAxisRotation('Z', value);
                    break;
            }
        }

        local_rotation.Normalize();
        pose.local_rotations[joint_index] = local_rotation;
    }

    return pose;
}

}  // namespace

BvhData BvhLoader::Load(const std::string& path) {
    TokenCursor cursor(ReadTokens(path));
    cursor.Expect("HIERARCHY");
    cursor.Expect("ROOT");

    BvhData data;
    const std::string root_name = cursor.Next();
    ParseJointBlock(cursor, data.skeleton, root_name, -1);

    if (data.skeleton.NumJoints() <= 0) {
        throw std::runtime_error("BvhLoader: skeleton contains no joints");
    }

    cursor.Expect("MOTION");
    cursor.Expect("Frames:");
    const int frame_count = cursor.NextInt();
    if (frame_count <= 0) {
        throw std::runtime_error("BvhLoader: frame count must be positive");
    }

    cursor.Expect("Frame");
    cursor.Expect("Time:");
    const float frame_time = cursor.NextFloat();
    if (frame_time <= 0.0f) {
        throw std::runtime_error("BvhLoader: frame time must be positive");
    }

    const int expected_channel_count = TotalChannelCount(data.skeleton);
    if (expected_channel_count <= 0) {
        throw std::runtime_error("BvhLoader: skeleton has no channels");
    }

    data.clip.name = path;
    data.clip.frames_per_second = 1.0f / frame_time;
    data.clip.frames.reserve(static_cast<std::size_t>(frame_count));

    for (int frame_index = 0; frame_index < frame_count; ++frame_index) {
        data.clip.frames.push_back(ParsePoseFrame(cursor, data.skeleton));
    }

    return data;
}

}  // namespace pmg
