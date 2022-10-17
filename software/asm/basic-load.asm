START      org       $0000
begin:
initaddr:  dc.l      $0000000000000080    ; Initial stack pointer
           dc.l      $0000000000000008    ; Initial PC
           move.b    #$aa,d0
           nop
           move.w    #$bbcc,d1
           nop
           move.l    #$aabbccdd,d2
           move.l    #$00000000,d3

           nop
           nop

           move.b    d0,$C0
           move.b    d0,$C1
           move.b    d0,$C2
           move.b    d0,$C3

           move.b    #01,$c0
           move.b    #02,$c1
           move.b    #03,$c2
           move.b    #04,$c3

           move.w    d1,$C0
           move.w    d1,$C1
           move.w    d1,$C2
           move.w    d1,$C3

           move.l    d2,$C0
           move.l    d3,$C0
           move.l    d3,$C4

           move.l    d2,$C1
           move.l    d3,$C0
           move.l    d3,$C4

           move.l    d2,$C2
           move.l    d3,$C0
           move.l    d3,$C4

           move.l    d2,$C3
           move.l    d3,$C0
           move.l    d3,$C4


           move.l    #$00ff00,d4
           movec     d4,VBR
           movec     d4,CAAR

           move.l    #$ffffffff,d4
           movec     d4,CACR
        ;    movec     #$00000000000000000,CACR

           move.b    d0,$100

       ; Repeat for ever
loop:
           jmp       loop

           dc.b      "Hello world"

           end       START
