//This overlay is used to draw messages in a box on top of the map widget
//Author: Bryan Poling
//Copyright (c) 2021 Sentek Systems, LLC. All rights reserved.
#pragma once

//External Includes
#include "../HandyImGuiInclude.hpp"

//Project Includes
#include "../EigenAliases.h"

class MessageBoxOverlay {
	private:
		std::mutex m_mutex;
		std::unordered_map<int, std::string> m_messages;
		int m_nextAvailableToken = 0;
		
	public:
		 MessageBoxOverlay() = default;
		~MessageBoxOverlay() = default;
		
		//Called in the draw loop for the map widget
		void Draw_MessageBox(Eigen::Vector2d const & CursorPos_NM, ImDrawList * DrawList, bool CursorInBounds);
		
		//Accessors for creating/deleting messages. Get a token before adding a message. When a message is added with a token that has already
		//been used, it replaces the previous message with that token. An added message will persist until it is removed or until Reset is called.
		void Reset();
		int  GetAvailableToken(void);
		void AddMessage(std::string const & Message, int Token);
		bool RemoveMessage(int Token);
};




