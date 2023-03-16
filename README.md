# bldc_for_bluepill
New BLDC adopted to use Makefile and run on Bluepill.

Here are some simple but key ideas for BLDC motor control:

When reading Hall sensors: Ha, Hb, Hc, they indicates 6 "H_positions"

	_Hc_Hb_Ha	H_pos
	=======================	
	001		H1
	010		H2
  	011		H3
  	100		H4
  	101		H5
  	110		H6



When a BLDC motor rotates in a direction, H_pos changes in sequence:

	Forward: H1 H3 H2 H6 H4 H5
	Bckward: H1 H5 H4 H6 H2 H3


A BLDC motor has three phases: Ia, Ib, Ic, arranged as follows:

	Ia -----+----- Ib
		|
		|
		Ic

The configuration can have the six actions if we assume for beginers that current flows from one phase to another:

	Motor Actions by Current Flow
	=============================
	Ia->Ib
	Ia->Ic
	Ib->Ic
	Ib->Ia
	Ic->Ia
	Ic->Ib


To rotate the motor into the desired neighbor H_pos, 
here are the transition between H_pos using Motor Actions:

		001	010	011	100	101	110
	=====================================================
	001			Ia->Ic		Ib->Ia
	010			Ia->Ic			Ic->Ib
	011	Ib->Ic	Ia->Ib
	100					Ib->Ia	Ic->Ib
	101	Ib->Ic			Ic->Ia
	110		Ia->Ib		Ic->Ia
	=====================================================

So a basic BLDC controller simply looks up a desired action to produce the desired movement from one H_pos into the next neighbor H_pos position.

That is all.

