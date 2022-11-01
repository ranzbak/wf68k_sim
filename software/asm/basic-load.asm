START      org       $0000
begin:
initaddr:  dc.l      $0000000000000080    ; Initial stack pointer
           dc.l      $0000000000000008    ; Initial PC
           move.b    #$aa,d0
           move.l    #$aabbccdd,d2
           move.l    #$00000000,d3

           nop
           nop

;            move.l    d0,$E0
;            move.l    $E0,$E4

; ; Check long move
;            move.l    $E4,d1
;            cmp.l     d0,d1
;            beq.s     .over
;            move.l    d0,$00ffcafe
; .over:     

; Check Byte move
           move.b    d0,$e0
           cmp.l     #$aa000000,$e0
           beq.s     .over2
           move.l    d0,$00ffd00d
.over2:


           move.b    d0,$e2
           cmp.l     #$aa00aa00,$e0
           beq.s     .over3
           move.l    d0,$00ffdead
.over3:

; Check word move ($bbcc)
           move.w    #$bbcc,$e4
           cmp.w     #$bbcc,$e4
           beq.s     .over4
           move.l    d0,$00ffd0d0
.over4:
; Check offset of word move in long word
           cmp.l     #$bbcc0000,$e4
           beq.s     .over5
           move.l    d0,$00ff0d0
.over5:   
         ;   move.w    d1,$C0
         ;   move.w    d1,$C1
         ;   move.w    d1,$C2
         ;   move.w    d1,$C3

         ;   move.l    d2,$C0
         ;   move.l    d3,$C0
         ;   move.l    d3,$C4

         ;   move.l    d2,$C1
         ;   move.l    d3,$C0
         ;   move.l    d3,$C4

         ;   move.l    d2,$C2
         ;   move.l    d3,$C0
         ;   move.l    d3,$C4

         ;   move.l    d2,$C3
         ;   move.l    d3,$C0
         ;   move.l    d3,$C4


           move.l    #$00ff00,d4
           movec     d4,VBR
           movec     d4,CAAR

           move.l    #$ffffffff,d4
           movec     d4,CACR
        ;    movec     #$00000000000000000,CACR

           move.b    d0,$00aa0000         ; Call trap to end simulation

       ; Repeat for ever
loop:
           jmp       loop

;hello_text:
           ;dc.b      "Hello world", 0

           ;end       START
