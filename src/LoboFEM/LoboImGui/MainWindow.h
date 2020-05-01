#pragma once
#include "imgui.h"
#include "imfilebrowser.h"

namespace Lobo
{
void ShowMainWindow(ImGui::FileBrowser *fileDialog, bool *p_open = NULL);
}