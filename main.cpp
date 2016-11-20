#include "cart_balance.h"


int main()
{
	// variables
	CB::Pendulum pend;

	// calculate all of the states of the pendulum along the curve
	pend.cycle();

	// initialize cart weight
	//Cart cart; // Object call cart of type cart
	//cart.x = 0;
	
	return 0;
}
