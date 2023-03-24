# BLDC_for_Bluepill
New BLDC adopted to use Makefile and run on Bluepill.

Here are some simple but key ideas for BLDC motor control:

When reading Hall sensors: Ha, Hb, Hc, they indicates 6 H_values or 6 H_physical_positions:

        HcHbHa       H_value   H_Phy_position
        ====================================
        001             H1      H_P0
        010             H2      H_P2
        011             H3      H_P1
        100             H4      H_P4
        101             H5      H_P5
        110             H6      H_P3



When a BLDC motor rotates in a direction, H_pos changes in sequence:

        Forward: H1 H3 H2 H6 H4 H5  (in terms of H_value)
                 P0 P1 P2 P3 P4 P5  (in terms of H physical position)

        Bckward: H5 H4 H6 H2 H3 H1  (in terms of H_value)
                 P5 P4 P3 P2 P1 P0  (in terms of H physical position)


A BLDC motor has three phases: Ia, Ib, Ic, arranged as follows:

	Ia -----+----- Ib
		|
		|
		Ic

The configuration can have the six actions if we assume for beginers that current flows from one phase to another:

        Motor Actions (Current Flow)    Index
        ======================================
        Ic->Ib                          A0
        Ia->Ib                          A1
        Ia->Ic                          A2
        Ib->Ic                          A3
        Ib->Ia                          A4
        Ic->Ia                          A5


To rotate the motor into the next desired neighbor H_pos or physical position, 
you apply the motor action as follows:

	now\next	P0	P2	P1	P4	P5	P3
	===================================================================
        P0				+A2		-A2
        P2				-A4			+A4
        P1		-A3	+A3
        P4						+A0	-A0
        P5		+A1			-A1
        P3			-A5		+A5
	====================================================================

So a basic BLDC controller simply looks up a desired action to produce the desired movement from 
one H_pos (or physical pos) into the next neighbor H_pos (or physical) position.

For example, from a P_now to go to P_next, the motor should apply the action as follows:
	
	P_now	(Hcba)	Action	P_next
	================================
	forward
	P0	(001)	(a->c)	P1
	P1	(011)	(b->c)	P2
	P2	(010)	(b->a)	P3
	P3	(110)	(c->a)	P4
	P4	(100)	(c->b)	P5
	P5	(101)	(a->b)	P0

	backward
	P5	(101)	-(a->b)	P4
	P4	(100)	-(c->b)	P3
	P3	(110)	-(c->a)	P2
	P2	(010)	-(b->a)	P1
	P1	(011)	-(b->c)	P0
	P0	(001)	-(a->c)	P5
	=================================

That is all, folks!
