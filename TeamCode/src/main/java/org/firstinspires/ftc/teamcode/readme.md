# TeamCode Lite

O projeto mantém o padrão modular do V4:

- `hardware`: constantes, configuração Pedro, composição do robô e transferência de pose.
- `commandbase/subsystem`: cada mecanismo e o controlador da sequência de bolas.
- `opmodes`: teleop de partida e autônomo mínimo de passagem de pose.
- `util`: matemática, seleção de alvo, previsão/estimativa e relógio monotônico.
- `samples`: setup de hardware e diagnósticos/tuning necessários para calibrar mecanismos.

Comece pelo [README_LITE.md](../../../../../../../../README_LITE.md), que lista os controles e os nomes de hardware. Mantenha valores ajustáveis em `hardware/GlobalConstants.java`, exceto as configurações Pedro de Mecanum, Pinpoint e Foresight, que ficam em `pedroPathing/Constants.java`.
