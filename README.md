# elevate-delta elevator application

The elevator application has the purpose to recognize if an elevator is moving 
and to provide this information to [accessibility.cloud](https://accessibility.cloud).

The project is a cooperation between [Sozialhelden](https://sozialhelden.de) and [Ubirch](https://ubirch.de).

Further information will follow soon.
  
## Configuration

you need to provide a `config.json` file, which includes:

```json
{
  "elevateDataUrl": "https://www.accessibility.cloud/equipment-status-reports.json",
  "elevateApiToken": "enter_token_here",
  "equipmentToken": "enter_token_here",
  "sourceId": "enter_id_here",
  "equipmentInfoId": "enter_info_here"
}
```