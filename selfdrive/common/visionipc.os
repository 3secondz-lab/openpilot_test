ELF          >                    8M          @     @   H�=    �    @ USH���  W�)D$P)D$@)D$0)D$ H���)D$H�D$`    H�|$h1��`  �    L��$�   H��$�   H�$H�T$�    �X   A�@   ���    �Ņ�xZ�D$�D$hD$D$pD$(�$�   D$8�$�   D$H�$�   D$X�$�   H�t$h�`  H���    � �    �8�    H��H�=    1�H���    ��H���  []�H��hH����D�HXA��A}[��L$@HD$X@HP(X8\$HT$8L$(D$H��\H�$    H�T$�   �X   I���    H��h�H�=    H�5    H�    �+   �     UAWAVAUATSH��H�L$H�t$I������   Lc�I�]1�@ H�{H��tGH�m    H�L�<�    M�L�4�   M�H�3�    H�C    �{��    �fff.�     L�{�I��H�D$D��E�H�D$H�pI�61��   �   E1��    H�CH���t H��H��L9��b���H��[A\A]A^A_]�H�=    H�5    H�    �D   �    @ UAWAVATSH��  I��A��A��H��W�CPC@C0C CH�C`    �C����H�=    �    �ŉ+���z  H��$�  1��`  �    Ǆ$�     D��$�  D��$�  �D$p   �$�  �$�  �$�  �$  D$x�$�   �$�   �$�   �$  �$�   L��$,  H�$    H�T$p�   �X   E1ɉ��    ����  �+W�)D$P)D$@)D$0)D$ )D$H�D$`    H�|$p1��`  �    L��$�   H��$�   H�$H�T$�    �X   A�@   ���    �Ņ��&  �D$�D$pD$D$xD$(�$�   D$8�$�   D$H�$�   D$X�$�   H��$0  H�t$p�`  �    ����   ��$0  ��   D9�$8  �  L��$8  AG@CPAAOAW A_0[@S0K C��$�  �kHc��   �    H�C`H����   H��$�  H��L�����    1�M��tcH��C@AF@KS [0A^0AV ANA�2�    �8�    H��H�=    1�H���    �;�    ����������H�Đ  [A\A^A_]�H�=    H�5    H�    �h   �    H�=    H�5    H�    �i   �    H�=    H�5    H�    �o   �    �     USH���  H���k����   H�|$h1��`  �    �D$h   �C�D$p�l$t�3�D$   D$p�$�   �$�   �$�   D$L$(T$8\$H�$�   D$XL��$�   H�$    H�T$�   �X   E1��    �C����H���  []�D  UAWAVAUATSH���  I��A�W�)D$P)D$@)D$0)D$ I��)D$H�D$`    H��$�   1��`  �    L��$�   H��$�   H�$H�T$�    �X   A�@   ���    ���a  �L$��$�   D$�$�   D$(�$�   D$8�$�   D$H�$�   D$X�$�   D��$�   D��$�   �$�   )D$p���  ���"  A�o����   H��$�   1�1��`  �    Ǆ$�      A�G��$�   ��$�   A�7�D$   �$�   �$�   �$�   �$�   D$L$(T$8\$H�$�   D$XL��$�   H�$    H�T$�   �X   E1��    ��~WE�gE�oE;o}}M��t(D$pAE�oIc�H�@H��I_`�&�    �8�    H��H�=    1�1�H���    �1�H��H���  [A\A]A^A_]�H�=    H�5    H�    ��   �    H�=    H�5    H�    ��   �    ff.�     AWAVSH���  I��A�^����   H�|$`1��`  �    �D$`   A�F�D$h�\$lA�6�D$   D$hL$x�$�   �$�   D$L$ T$0\$@�$�   D$PL��$�   H�$    H�T$�   �X   E1��    A�F����A�NI�~`��~RE1��   �    H�H��t*H�t�H���    I�F`H�    �|��    A�NI�~`I��Hc�H��I9�|�H��t�    A�>��x�    H���  [A^A_�/tmp/vision_socket vipc_recv err: %s
 p2->num_fds <= VIPC_MAX_FDS selfdrive/common/visionipc.c int vipc_send(int, const VisionPacket *) bufs[i].addr != MAP_FAILED void vipc_bufs_load(VIPCBuf *, const VisionStreamBufs *, int, const int *) rp.type == VIPC_STREAM_BUFS int visionstream_init(VisionStream *, VisionStreamType, _Bool, VisionStreamBufs *) rp.d.stream_bufs.type == type s->bufs rp.type == VIPC_STREAM_ACQUIRE VIPCBuf *visionstream_get(VisionStream *, VIPCBufExtra *) s->last_idx < s->num_bufs clang version 6.0.0-1ubuntu2 (tags/RELEASE_600/final) selfdrive/common/visionipc.c /home/rnd/openpilot_test VISION_STREAM_RGB_BACK VISION_STREAM_RGB_FRONT VISION_STREAM_RGB_WIDE VISION_STREAM_YUV VISION_STREAM_YUV_FRONT VISION_STREAM_YUV_WIDE VISION_STREAM_MAX VisionStreamType VIPC_INVALID VIPC_STREAM_SUBSCRIBE VIPC_STREAM_BUFS VIPC_STREAM_ACQUIRE VIPC_STREAM_RELEASE VisionIPCPacketType int vipc_connect vipc_send fd p2 type d stream_sub tbuffer _Bool stream_bufs width height stride buf_len long unsigned int size_t buf_info ui_info big_box_x big_box_y big_box_width big_box_height transformed_width transformed_height front_box_x front_box_y front_box_width front_box_height wide_box_x wide_box_y wide_box_width wide_box_height VisionUIInfo VisionStreamBufs stream_acq idx extra frame_id unsigned int __uint32_t uint32_t timestamp_eof __uint64_t uint64_t VIPCBufExtra stream_rel VisionPacketData num_fds fds sizetype VisionPacket ret p VisionPacketWire vipc_recv out_p vipc_bufs_load visionstream_init visionstream_release visionstream_get len addr VIPCBuf visionstream_destroy bufs i s ipc_fd last_idx last_type num_bufs bufs_info VisionStream out_bufs_info err rp rep out_extra        5        U5               V       �        T                       3        T3              S                �              V                             U      y       T~      �       T                             T      Z       P~      �       P                y      ~       P                �      �       U�      w       ]}      �       ]                �      �       T�      r       w}      �       w                �             Q      '       Q                �      �       R�      r       w}      �       w                �      �        ��      a       V}      �       V                �      �       U�             SI      {       S�      �       S                �      �       T�      �       _I      {       _�      �       _                �      �       Q�      �       \�      v       \�      �       \                �      �       R�      �       ^�      �       ^                �      �       V                      v       w��      �       w�                +      �       w�                �      �       P                �      �       P      �       VI      i       V�      �       V                �             V                      �       VI      i       V                �      �       U�      �       S                      �       w�                 -      �       T                -      �       w�                 �      �       U�      �       _�      	       _                �      �       T�      �       ^�      	       ^                �      �       S�      �       S                3      �       P�      �       P                3      �       Pg      �       P�      �       P�      �       P�      	       P                �      k       w�                �      g       T                �      k       w�                g      k       P                 	      /	       U/	      S
       ^                @	      �	       w�                 c	      �	       T                c	      �	       w�                 �	      #
        �#
      '
       _                %�B  :;  (    I  $ >     . @1  .@1  	 1  
4 1  4 1  .@:;'?   :;I  U  4 :;I  . :;'I?   .:;'I?    :;I  4 :;I  & I   I:;  :;   I:;8  :;  :;  :;  :;  I  ! I7  $ >  .@:;'I?    1XY  !1XY  "1UXY  #   >                                V
                                                              �                  Wx             W�  	    �  	    �  
    �  �  �           �   W�  	    �  	    �  
    �  �           �   W    6        6�          6�          7�           7�              8�         �       *�       *�       *�      1�       -r   �  �  �      U    `P    �   Q       R    �   SX    _  T\       N    P?    +  C @    �  A     �  B     �  D     `  I E    �  F     �   G       H     �  M J    �  K     �   L  *           �      7    P.    �  /     �   1    �   1    �   1    2  2       684    D  5   =      >    O      ,    8!    �   "     �   "    �   #    �   #    �   $    �   $    �   &    �   &    �   '     �   '$    �   )(    �   ),    �   *0    �   *4       =    9    ,  ;     I  < 7      B      )    T      =      ,�   k  @     }          X    �               �       �       �      �       r      �   �          H  W    I�           I�          I�          I�          I7          S�          J�       a�   x             P!�          ~   Z	    �  	    �  
    �   "�      b	    �  
    �            �   W    z        z�      {�   #        �           }�  !�          m   �	    �  	    �             e  W    ��          ��          �<          ��       ��  "�      �	    �  
    �   #        �           ��  !�          p   �	    �  	    �  
    �             6  W    �        ��      ��   #        �           ��  !�          j   �	    �  	    �    #        ^           ��     �  �      _    [    �   \     2  ]    �   ^ �  �  �  �   �  �      l    he    �   f     �   g    �   h    �   i    �  j    �  k` �      �      n      }      �                      �      �      I      i                      �      �      �      �                       �        B    vipc_bufs_load �  vipc_recv 9  visionstream_get �  visionstream_release �  visionstream_init x  vipc_connect �  vipc_send �  visionstream_destroy     F       B  O  VisionUIInfo �  VisionStream �  VisionStreamType }  VisionPacketWire B  unsigned int   VisionPacketData �   int �  _Bool ]   VisionIPCPacketType 2  size_t �  VisionPacket �  VIPCBuf   VIPCBufExtra �  VisionStreamBufs 7  __uint32_t =  long unsigned int ,  uint32_t T  __uint64_t I  uint64_t      clang version 6.0.0-1ubuntu2 (tags/RELEASE_600/final)        zR x�                      0           AAG���    P       �    Dp0   d       �    ABB B(B0A8DP������  ,   �       H   ABB B(A0G�	�����     �       �    AAG��� 0   �       e   ABB B(B0A8G������� $         6   BBA G����    J   �   �      selfdrive/common /usr/lib/llvm-6.0/lib/clang/6.0.0/include /usr/include/x86_64-linux-gnu/bits  visionipc.h   visionipc.c   stddef.h   types.h   stdint-uintn.h     	        

 �
d#./C�]��".1JKC,�#t� �
��1-LI-J�xX "
�./	J<G.$:X�Y�<1s� =�=<NJ�.8Jtf� #
�"
It9�/.�f� ���X��IM�*�.m�.*C�]��".iJKK?�.k�g��f� �,t=<�K<;h��<�.� J,�-#t�`�  p��� '

�<.�f� X��;��.�I>�� �v �
@�<0C�]���.iJuA� �.i<
hJ.�~f����I��<�IA�� �OKKJ0<"/�J,�~#t�`J�.gJ� 	*

�J.�~f�X��I��~<�I;�����.�~.��	J<*/X
�J�JT!�J�5<.
Y<..Y                              ��                                   r                  i    &              `    B              W    �              N    �              E    ]             <    {             3    �             *    �             �    �       K       �    _       )       x     �      :       I     
      S                                                                                                                          
                                       R                     A                     �                     w                     �                     �                      �                                            q                                                                p                                          �    �      �       �                    +                   �          �             	      6      �     �      e      _     �      H      }    �      �                    ��������             ��������P             ���������             ���������             ���������             ���������          #   ���������             ��������          "   ��������u            ���������            ���������            ���������            ���������            ���������         !   ��������            ��������O             ���������            ���������            ���������            ���������            ���������            ���������            ��������            ���������            ���������            ��������            ��������|            ���������            ��������         $   ��������J            ��������Q         #   ��������[            ��������e         "   ��������l            ���������            ���������            ���������            ���������            ���������            ���������            ���������            ���������            ���������         	   ���������            ���������            ���������            ��������            ���������            ���������            ��������/            ���������            ��������c            ���������            ���������         #   ���������            ���������         "   ���������         
   ���������            ���������            ���������            ���������            �������� 	            ��������	            ��������	            ��������H	            ���������	            ��������
         !   ��������
            ��������9
            ��������E
            ��������       
                     
                     
      6              
                     
      S                            +       
            3       
      l       9       
      �       ?       
      �       E       
      �       K       
      �       Q       
      �       W       
      �       ^       
      r      f       
            l       
      #      r       
      9      x       
      J      ~       
      ^      �       
      �      �                     �                    �       
              �       
      I       �       
             �                   �       
      �       �       
      �             
      4                  �             
      �      '      
      W      +      
      ;      6      
      �      :      
      �      E      
      �      I      
      �      T      
      !      X      
      �      c      
              h      
      l      l      
      @      y      
      �      �      
      �      �      
      �      �      
      �      �      
      �      �      
      �      �      
      �      �      
      �      �      
      �      �      
      �      �      
      �             
      �            
      t            
      t             
      �      0      
      �      <      
      �      I      
      �      U      
      �      e      
      �      q      
            }      
      
      �      
      i      �      
      �      �      
            �      
            �      
      �      �      
      �      �      
      �      �      
      �      �      
      �      �      
      �      �      
      �      	      
      �            
            %      
            7      
            >      
      �      I      
      �      P      
      �      X      
            d      
      #      p      
      -      |      
      ;      �      
      J      �      
      \      �      
      o      �      
      {      �      
      �      �      
      �      �      
      �      �      
      �      �      
      �      �      
      �            
      \            
      \            
                   
      :      1      
      1      <      
      &      C      
            N      
      S      Y      
      H      l      
      �      w      
      �      ~      
      �      �      
      �      �      
      �      �      
      �      �      
      �      �      
      �      �      
      �      �      
      �      �      
      �      �            �      �      
      �            
      �            
      B            
                  
      �            
      o      #      
      �      .      
      �      2      
      ~      =      
      7      A      
      �      L      
      �      P      
      �      [      
      �      j            �      }            +      �      
            �      
      q      �      
      �      �      
      0       �      
            �      
      8      �            �      �      
      �      �      
      n      �      
      B      �      
      �      �                        
      �            
      �                  -      %      
      �      .      
      �      :            �      H      
            S      
            W      
      B      b      
      Z      f      
      �      q      
            u      
      �      �      
      �      �      
      `       �      
      �      �      
      �      �            �      �      
      ~      �      
      �      �            �      �      
      �      �      
      �      �      
      �      �             	            
      &            
                  
      B            
      �      %            @	      2      
      D      6      
      �      E            c	      T      
      i      ]      
      �      h            �	      u      
      �      y      
      @      �      
            �      
            �      
      �      �      
            �      
            �      
      q      �      
      q      �      
      D      �      
      K            
      T            
      ^            
      g      +      
      ;             
                     
                                    4                    T                   h             �      �             �      �             �      �             �                    	      �                      memcpy visionstream_destroy .debug_abbrev vipc_recv .rela.text .comment .L__PRETTY_FUNCTION__.visionstream_init memset .L__PRETTY_FUNCTION__.visionstream_get vipc_connect .rela.debug_pubtypes .rela.debug_pubnames .debug_ranges ipc_sendrecv_with_fds .debug_str .L.str strerror munmap mmap .debug_macinfo .rela.debug_info __errno_location __assert_fail .note.GNU-stack printf close visionstream_release .rela.debug_line .rela.eh_frame free .L__PRETTY_FUNCTION__.vipc_send .L__PRETTY_FUNCTION__.vipc_bufs_load calloc .debug_loc visionipc.c .strtab .symtab .L.str.9 .L.str.8 .L.str.7 .L.str.6 .L.str.5 .L.str.4 .L.str.3 .L.str.2 .L.str.1 .rodata.str1.1                                                                                            �J      �                             :                     @       V
                             5                      �0      �                          {     2               �
      �                            �      0               �      �                                                 -      �                                                         �                             5                     �      B                             0                     (7      x                          �                      $      �                              !                     �$                                    �                      �$      �                              �                      �I                                 �                      O%      J                             �                      �I                                 @      0               �&      7                             `                     �&                                     �    p               �&      @                             �                     �I      �                           �                     (      N                             �                     �J                                 "                     `,                                 