import React, { useState } from 'react';
import {
  Card, Container, useTheme, Typography, CardContent, CardActions, Paper, TextareaAutosize, Alert,
} from '@mui/material';
import IconButton from '@mui/material/IconButton';
import Box from '@mui/material/Box';
import InputLabel from '@mui/material/InputLabel';
import MenuItem from '@mui/material/MenuItem';
import FormControl from '@mui/material/FormControl';
import Select from '@mui/material/Select';
import QrCodeIcon from '@mui/icons-material/QrCode';
import UploadFileIcon from '@mui/icons-material/UploadFile';
import AddIcon from '@mui/icons-material/Add';
import RemoveIcon from '@mui/icons-material/Remove';
import AutoGraphIcon from '@mui/icons-material/AutoGraph';
import DeleteIcon from '@mui/icons-material/Delete';
import TextField from '@mui/material/TextField';
import Grid from '@mui/material/Unstable_Grid2';
import ProfileChart from '../../components/chart/ProfileChart';
import { Profile } from '../../models/profile';

export default function Profiles() {
  const theme = useTheme();
  const [error, setError] = useState(null);
  const [inputList, setInputList] = useState([]);
  const [profile, setProfile] = useState(new Profile([]));

  const updateProfile = (value) => {
    try {
      setProfile(Profile.parse(JSON.parse(value)));
      setError(undefined);
    } catch (er) {
      setError(er.message);
    }
  };

  const addProfileStep = (event) => {
    setProfile(event.target.value);
  };

  const remProfileStep = () => {
    setInputList(inputList.slice(0, -5));
    setProfile('');
  };

  const remProfile = () => {
    setInputList(inputList.slice(-1, -1));
    setProfile('');
  };

  const handleButtonClick = () => {
    setInputList([...inputList,
      <br></br>,
      <Box
        sx={{
          // minWidth: 120,
          mr: theme.spacing(2),
          mt: theme.spacing(2),
        }}
      >
        <FormControl focused size>
          <InputLabel id="demo-simple-select-label">Type</InputLabel>
          <Select onChange={addProfileStep} labelId="phase-select" key={inputList.length} size="small" id="phase-type-select" value={profile} label="Type" variant="outlined">
            <MenuItem value={1}>Pressure</MenuItem>
            <MenuItem value={2}>Flow</MenuItem>
          </Select>
        </FormControl>
      </Box>,
      <TextField id="outlined-basic" key={inputList.length + 1} size="small" label="Bar" variant="outlined" sx={{ mr: theme.spacing(2), mt: theme.spacing(1) }} />,
      <TextField id="outlined-basic" key={inputList.length + 1} size="small" label="Flow" variant="outlined" sx={{ mr: theme.spacing(2), mt: theme.spacing(1) }} />,
      <TextField id="outlined-basic" key={inputList.length + 1} size="small" label="Time" variant="outlined" sx={{ mr: theme.spacing(2), mt: theme.spacing(1) }} />,
    ]);
  };

  return (
    <div>
      <Container sx={{ mt: theme.spacing(2) }}>
        <Card sx={{ mt: theme.spacing(2) }}>
          <Grid container columns={{ xs: 1, sm: 2 }}>
            <Grid item xs={1}>
              <CardContent>
                <Typography gutterBottom variant="h5">
                  Load Profile
                </Typography>
              </CardContent>
              <CardActions>
                <IconButton style={{ float: 'right' }} color="primary" aria-label="upload picture" component="label" sx={{ ml: theme.spacing(3) }}>
                  <input hidden accept=".bin" type="file" />
                  <UploadFileIcon fontSize="large" />
                </IconButton>
                <IconButton style={{ float: 'right' }} color="primary" aria-label="upload picture" component="label">
                  <input hidden accept=".png" type="file" />
                  <QrCodeIcon fontSize="large" />
                </IconButton>
              </CardActions>
            </Grid>
          </Grid>
        </Card>
      </Container>
      <Container sx={{ mt: theme.spacing(2) }}>
        <Card sx={{ mt: theme.spacing(2) }}>
          <Grid container columns={{ xs: 1, sm: 1 }}>
            <Grid item xs={1}>
              <CardContent>
                <Typography gutterBottom variant="h5">
                  Build Profile
                  <IconButton style={{ float: 'right' }} onClick={remProfile} color="primary" aria-label="upload picture" component="label" sx={{ ml: theme.spacing(3) }}>
                    <DeleteIcon fontSize="large" />
                  </IconButton>
                  <IconButton style={{ float: 'right' }} onClick={remProfileStep} color="primary" aria-label="upload picture" component="label" sx={{ ml: theme.spacing(3) }}>
                    <RemoveIcon fontSize="large" />
                  </IconButton>
                  <IconButton style={{ float: 'right' }} onClick={handleButtonClick} color="primary" aria-label="upload picture" component="label" sx={{ ml: theme.spacing(3) }}>
                    <AddIcon fontSize="large" />
                  </IconButton>
                  <IconButton style={{ float: 'right' }} color="primary" aria-label="upload picture" component="label" sx={{ ml: theme.spacing(3) }}>
                    <AutoGraphIcon fontSize="large" />
                  </IconButton>
                  {inputList}
                </Typography>
              </CardContent>
            </Grid>
          </Grid>
        </Card>
      </Container>
      <Container sx={{ mt: theme.spacing(2) }}>
        <Paper sx={{ mt: theme.spacing(2), p: theme.spacing(2) }}>
          <Typography variant="h5" sx={{ mb: theme.spacing(2) }}>
            Profile syntax playground
          </Typography>
          <Grid container columns={{ xs: 1, sm: 3 }} spacing={2}>
            <Grid item xs={1} sm={3}>
              <Alert severity={error ? 'error' : 'success'}>
                {error || 'Nice syntax!'}
              </Alert>
            </Grid>
            <Grid item xs={1} sm={1}>
              <TextareaAutosize
                minRows={15}
                onChange={(evt) => updateProfile(evt.target.value)}
                style={{ width: '100%', backgroundColor: theme.palette.grey[200] }}
              >
              </TextareaAutosize>
            </Grid>
            <Grid item xs={1} sm={2} position="relative" height="400">
              <ProfileChart profile={profile} />
            </Grid>
          </Grid>
        </Paper>
      </Container>
    </div>
  );
}
