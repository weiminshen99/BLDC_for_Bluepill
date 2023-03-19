# BLDC_for_Bluepill
New BLDC adopted to use Makefile and run on Bluepill.

Here are some simple but key ideas for BLDC motor control:

When reading Hall sensors: Ha, Hb, Hc, they indicates 6 "H_positions" or 6 physical positions:

        _Hc_Hb_Ha       H_pos   Phy_position
        ====================================
        001             H1      P0
        010             H2      P2
        011             H3      P1
        100             H4      P4
        101             H5      P5
        110             H6      P3



When a BLDC motor rotates in a direction, H_pos changes in sequence:

        Forward: H1 H3 H2 H6 H4 H5  (in terms of H_pos)
                 P0 P1 P2 P3 P4 P5  (in terms of physical position)

        Bckward: H5 H4 H6 H2 H3 H1  (in terms of H_pos)
                 P5 P4 P3 P2 P1 P0  (in terms of physical position)


A BLDC motor has three phases: Ia, Ib, Ic, arranged as follows:

	Ia -----+----- Ib
		|
		|
		Ic

The configuration can have the six actions if we assume for beginers that current flows from one phase to another:

        Motor Actions (Current Flow)    Index
        ======================================
        Ib->Ic                          A0
        Ib->Ia                          A1
        Ic->Ia                          A2
        Ic->Ib                          A3
        Ia->Ib                          A4
        Ia->Ic                          A5


To rotate the motor into the next desired neighbor H_pos or physical position, 
you apply the motor action as follows:

        now\next        001/P0  010/P2  011/P1  100/P4  101/P5  110/P3
        ================================================================
        001/P0                          (c->a)		-(c->a)
        010/P2                          -(a->b)			(a->b)
        011/P1		-(c->b)	(c->b)
        100/P4                                          (b->c)	-(b->c)
        101/P5          (b->a)			-(b->a)
        110/P3                  -(a->c)		(a->c)
        ================================================================

So a basic BLDC controller simply looks up a desired action to produce the desired movement from 
one H_pos (or physical pos) into the next neighbor H_pos (or physical) position.

For example, to go to a desired next H_pos or physical position, the motor should apply the action as follows:

        P_now	P_next	Action
        =========================
        P0	P1	(c->a)
        P0	P5	-(c->a)
	...	...	...
        P3	P4	(a->c)
        P3	P2	-(a->c)
	...	...	...
        P5	P0	(b->a)
        P5	P4	-(b->a)

That is all, folks!
