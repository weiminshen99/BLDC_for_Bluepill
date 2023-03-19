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
        Ic->Ia                          A0
        Ic->Ib                          A1
        Ia->Ib                          A2
        Ia->Ic                          A3
        Ib->Ic                          A4
        Ib->Ia                          A5


To rotate the motor into the next desired neighbor H_pos or physical position, 
you apply the motor action as follows:

        now\next        001/P0  010/P2  011/P1  100/P4  101/P5  110/P3
        ===============================================================
        001/P0                          A3              A5
        010/P2                          A3                      A1
        011/P1          A4      A2
        100/P4                                          A5      A1
        101/P5          A4                      A0
        110/P3                  A2              A0
        ============================================================

So a basic BLDC controller simply looks up a desired action to produce the desired movement from one H_pos into the next neighbor H_pos position.

For example, to go to a desired next H_pos or physical position, the motor should apply the action as follows:

        Hx/Px(next)     Action
        =========================
        001/P0          A4 (b->c)
        010/P2          A2 (a->b)
        100/P1          A1 (c->b)
        011/P4          A3 (a->c)
        101/P5          A5 (b->a)
        100/P3          A0 (c->a)

That is all, folks!
