# QUICK_IA_BIOBUZZ_Lite

Projeto FTC simplificado a partir do branch `V4` do `QUICK_IA_BIOBUZZ`. As bibliotecas SolversLib e Pedro Pathing, a configuração Mecanum/Pinpoint/Foresight e os valores atuais de hardware foram preservados.

## Estrutura principal

- `TeamCode/.../hardware/GlobalConstants.java`: nomes, sentidos, potências, servos, sensor, alvos e poses.
- `TeamCode/.../hardware/MyRobot.java`: ponto único que monta os subsistemas.
- `TeamCode/.../hardware/AutoPoseHandoff.java`: transferência de uso único da pose entre OpModes.
- `TeamCode/.../pedroPathing/Constants.java`: configuração Pedro Mecanum, Pinpoint e Foresight.
- `TeamCode/.../commandbase/subsystem/`: drive, intake, transfer, torre, shooter, hood, ball gate, flower collector, sensor e feedback Blinkin.
- `TeamCode/.../opmodes/TeleOp_All.java`: teleop de partida e controles individuais.
- `TeamCode/.../opmodes/Auto_PoseHandoff.java`: rota simples que entrega a pose final ao TeleOp.
- `TeamCode/.../util/`: matemática, seleção de alvo, estimativa de movimento, previsão de mira e relógio monotônico.

## Controles

| Gamepad | Controle | Ação |
| --- | --- | --- |
| 1 | Sticks | Dirigir o chassi |
| 1, INIT | D-pad esquerdo/direito | Selecionar BLUE/RED |
| 1 | A | Coleta normal com intake e transfer |
| 1 | X | Descarregar com intake e transfer em reverso |
| 1 | B | Parar mecanismos e fechar a trava |
| 1 | Y | Preparar e iniciar o tiro quando shooter e torre estiverem prontos |
| 1 | RB | Alternar mira automática da torre |
| 1 | BACK | Redefinir a pose do robô para `(135 in, 81 in, 90°)` |
| 1 | START | Gravar o zero frontal da torre com chassi e torre parados e mira desligada |
| 2 | A/B/X | Intake para frente / reverso / parar |
| 2 | Y/LB/RB | Transfer para frente / reverso / parar |
| 2 | RT/LT | Shooter para frente / parar |
| 2 | D-pad cima/baixo | Implantar / recolher flower collector |
| 2 | D-pad direita/esquerda | Abrir / fechar ball gate |
| 2 | Stick esquerdo horizontal | Controle manual da torre; desliga a mira automática |
| 2 | START/BACK | Ajustar o hood em passos pequenos |

O alvo da torre acompanha a aliança selecionada e muda para a coordenada refletida ao cruzar a linha central, com histerese para reduzir oscilações. A mira automática só se move depois de um zero frontal válido. `BACK` redefine a pose do Pinpoint/Pedro para a localização fixa; `START` é a ação separada que grava o zero do encoder da torre.

## Mecanismos e valores migrados

- Intake e transfer mantêm nomes, inversões, frenagem e potências do V4.
- Shooter mantém potência, encoder SolversLib e setpoint/PIDF de `ShooterTuningConfig`.
- Ball gate mantém servo, posições `0.55` (fechada) e `0.80` (aberta), e a mesma lógica de bloqueio.
- Flower collector usa os dois servos espelhados e as posições configuradas no V4.
- A torre usa o encoder integrado do REV HD Hex como leitura-base. O REV Through Bore ligado ao canal de encoder `intake` fornece a leitura de verificação e corrige o ângulo usado pela mira.
- O sensor REV da entrada da rampa conta passagens estáveis e para a coleta ao alcançar quatro bolas. O Blinkin indica coleta, carregador cheio, shooter pronto, tiro e falha de sensor.

O V4 não guarda reduções dos motores de intake/transfer/shooter como números no Java; os valores de velocidade programados e os nomes dos motores foram mantidos como encontrados. O HD Hex usa 28 contagens por volta do eixo do motor. A escala da torre no novo código considera a caixa 3:1 e reutiliza a relação externa medida no V4 (`20.025` contagens de Through Bore por volta de saída). Confirme a diferença dos encoders na bancada antes de habilitar mira automática.

## Configuração de hardware que deve coincidir

`GlobalConstants.java` e `pedroPathing/Constants.java` contêm os nomes do Robot Configuration. O dispositivo `intake` fornece energia ao mecanismo intake e sua entrada de encoder é usada para ler o Through Bore; conecte esse encoder à porta de encoder associada a esse dispositivo. O sensor de distância continua sendo `magazineInlet`, na entrada da rampa. Não remapeie esses nomes sem atualizar as constantes.

## Construção

Abra a pasta no Android Studio e execute a tarefa de montagem do módulo FTC. As dependências existentes do FTC SDK, SolversLib, Pedro Pathing e Panels permanecem nos Gradle files originais; não altere esses arquivos como parte dos ajustes de mecanismo.
