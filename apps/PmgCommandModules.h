#pragma once

#include <optional>

namespace pmgcli {

// Command modules return an exit code when they recognize argv[1].
// Unrecognized commands return nullopt so the dispatcher can try next module.
std::optional<int> TryRunBvhCommand(int argc, char** argv);
std::optional<int> TryRunTransitionCommand(int argc, char** argv);
std::optional<int> TryRunGraphCommand(int argc, char** argv);
std::optional<int> TryRunDiagnosticCommand(int argc, char** argv);
std::optional<int> TryRunRuntimeCommand(int argc, char** argv);

}  // namespace pmgcli
