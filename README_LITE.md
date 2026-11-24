# QUICK_IA_BIOBUZZ_Lite

Projeto FTC simplificado a partir do branch `V4` do `QUICK_IA_BIOBUZZ`. As bibliotecas SolversLib e Pedro Pathing, a configuração Mecanum/Pinpoint/Foresight e os valores atuais de hardware foram preservados.

## Estrutura principal

- `TeamCode/.../hardware/GlobalConstants.java`: nomes, sentidos, potências, servos, sensor, alvos e poses.
- `TeamCode/.../hardware/MyRobot.java`: ponto único que monta os subsistemas.
- `TeamCode/.../hardware/AutoPoseHandoff.java`: transferência de uso único da pose entre OpModes.
- `TeamCode/.../pedroPathing/Constants.java`: configuração Pedro Mecanum, Pinpoint e Foresight.
- `TeamCode/.../commandbase/subsystem/`: drive, intake, transfer, torre, shooter, ball gate, flower collector, sensor e feedback Blinkin.
- `TeamCode/.../opmodes/TeleOp_All.java`: teleop completo para um operador no Gamepad 1.
- `TeamCode/.../opmodes/Auto_PoseHandoff.java`: rota simples que entrega a pose final ao TeleOp.
- `TeamCode/.../util/`: matemática, seleção de alvo e relógio monotônico.

O TeleOp vincula os botões do Gamepad 1 a `InstantCommand`s do SolverLib, com requisitos dos subsistemas envolvidos. A preparação e o disparo formam uma sequência de comandos; direção, mira automática e feedback usam comandos contínuos do scheduler.

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
| 1 | D-pad cima/baixo | Implantar / recolher flower collector |
| 1 | D-pad direita/esquerda | Abrir / fechar ball gate |
| 1, segurar LB | A/B/X | Intake para frente / reverso / parar |
| 1, segurar LB | D-pad direita/esquerda/baixo | Transfer para frente / reverso / parar |
| 1, segurar LB | RT/LT | Shooter para frente / parar |
| 1, segurar LB | Stick direito horizontal | Controle manual da torre; desliga a mira automática |

O alvo da torre acompanha a aliança selecionada e muda para a coordenada refletida ao cruzar a linha central, com histerese para reduzir oscilações. A mira usa a pose atual estimada pelo Pinpoint, sem prever movimento futuro. A mira automática só se move depois de um zero frontal válido. `BACK` redefine a pose do Pinpoint/Pedro para a localização fixa; `START` é a ação separada que grava o zero do encoder da torre.

## Mecanismos e valores migrados

- Intake e transfer mantêm nomes, inversões, frenagem e potências do V4.
- Shooter mantém potência, encoder SolversLib e setpoint/PIDF de `ShooterTuningConfig`.
- Ball gate mantém servo, posições `0.55` (fechada) e `0.80` (aberta), e a mesma lógica de bloqueio.
- Flower collector usa os dois servos espelhados e as posições configuradas no V4.
- A torre usa o encoder integrado do REV HD Hex como leitura-base. O REV Through Bore ligado ao canal de encoder `intake` fornece a leitura de verificação e corrige o ângulo usado pela mira.
- O sensor REV da entrada da rampa conta passagens estáveis e para a coleta ao alcançar quatro bolas. O Blinkin indica coleta, carregador cheio, shooter pronto, tiro e falha de sensor.

O V4 não guarda reduções dos motores de intake/transfer/shooter como números no Java; os valores de velocidade programados e os nomes dos motores foram mantidos como encontrados. O encoder integrado da torre usa os **497,896 ticks por volta de saída já calibrados**. O Through Bore ainda não foi calibrado: sua escala inicial é calculada com 8.192 counts/rev e a engrenagem de 35 dentes no anel de 85 dentes (`8192 × 85 / 35 ≈ 19.894,86 ticks por volta da torre`). Use `TurretThroughBoreDiagnosticOpMode` para ajustar essa escala e o sinal antes de confiar na correção externa.

## Configuração de hardware que deve coincidir

`GlobalConstants.java` e `pedroPathing/Constants.java` contêm os nomes do Robot Configuration. O dispositivo `intake` fornece energia ao mecanismo intake e sua entrada de encoder é usada para ler o Through Bore; conecte esse encoder à porta de encoder associada a esse dispositivo. O sensor de distância continua sendo `magazineInlet`, na entrada da rampa. Não remapeie esses nomes sem atualizar as constantes.

## Construção

Abra a pasta no Android Studio e execute a tarefa de montagem do módulo FTC. As dependências existentes do FTC SDK, SolversLib, Pedro Pathing e Panels permanecem nos Gradle files originais; não altere esses arquivos como parte dos ajustes de mecanismo.
