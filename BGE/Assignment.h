#pragma once
#include "Game.h"

namespace BGE
{
	class Assignment :
		public Game
	{
	public:
		Assignment(void);
		~Assignment(void);
		bool Initialise();
		void Update();

	};

}