#pragma once

// Test prelude: on Windows debug builds, route failed assert()/abort() to
// stderr instead of opening a modal dialog, so the test suite fails fast and
// non-interactively. Include this first in every test translation unit.
#ifdef _WIN32
#include <crtdbg.h>
#include <cstdlib>

namespace {

struct CrtDialogSilencer {
    CrtDialogSilencer() {
        _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTDLG);
        for (int report_type : {_CRT_ASSERT, _CRT_ERROR, _CRT_WARN}) {
            _CrtSetReportMode(report_type, _CRTDBG_MODE_FILE);
            _CrtSetReportFile(report_type, _CRTDBG_FILE_STDERR);
        }
    }
};

// Constructed before main() runs, so the redirect is in place for all asserts.
const CrtDialogSilencer g_crt_dialog_silencer;

}  // namespace
#endif  // _WIN32
